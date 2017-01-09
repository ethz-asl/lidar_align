#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <algorithm>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "lidar_align/aligner.h"
#include "lidar_align/sensors.h"
#include "lidar_align/table.h"

// number of frames to take when calculating rough 2D alignment
constexpr int kDefaultUseNScans = 100000000;
constexpr float kDefaultMininumAngularVelocity = 0.0;

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_align");

  ros::NodeHandle nh, nh_private("~");

  std::string input_bag_path;
  if (!nh_private.getParam("input_bag_path", input_bag_path)) {
    ROS_FATAL("Could not find input_bag_path parameter, exiting");
    exit(EXIT_FAILURE);
  }

  int use_n_scans;
  nh_private.param("use_n_scans", use_n_scans, kDefaultUseNScans);

  float minium_angular_velocity;
  nh_private.param("minium_angular_velocity", minium_angular_velocity,
                   kDefaultMininumAngularVelocity);

  rosbag::Bag bag;
  bag.open(input_bag_path, rosbag::bagmode::Read);

  std::vector<std::string> types;
  types.push_back(std::string("sensor_msgs/PointCloud2"));
  types.push_back(std::string("geometry_msgs/TwistStamped"));
  types.push_back(std::string("geometry_msgs/TransformStamped"));
  rosbag::View view(bag, rosbag::TypeQuery(types));

  std::vector<std::string> column_names;
  column_names.push_back("x");
  column_names.push_back("y");
  column_names.push_back("z");
  column_names.push_back("rx");
  column_names.push_back("ry");
  column_names.push_back("rz");

  std::shared_ptr<Table> table_ptr =
      std::make_shared<Table>(column_names, 20, 10);

  Scan::Config scan_config;
  nh_private.param("min_point_distance", scan_config.min_point_distance,
                   scan_config.min_point_distance);
  nh_private.param("max_point_distance", scan_config.max_point_distance,
                   scan_config.max_point_distance);
  nh_private.param("keep_points_ratio", scan_config.keep_points_ratio,
                   scan_config.keep_points_ratio);

  LidarArray lidar_array;
  Odom odom;

  size_t scan_num = 0;
  size_t reject_num = 0;
  for (const rosbag::MessageInstance& m : view) {
    if (m.getDataType() == std::string("sensor_msgs/PointCloud2")) {
      Scalar angular_velocity;
      if (!odom.getFinalAngularVeloctiy(&angular_velocity) ||
          (angular_velocity < minium_angular_velocity)) {
        reject_num++;
        continue;
      }

      std::stringstream ss;
      ss << "Loading scan:       " << scan_num++ << " Rejected:         " << reject_num;
      table_ptr->updateHeader(ss.str());

      Pointcloud pointcloud;
      pcl::fromROSMsg(*(m.instantiate<sensor_msgs::PointCloud2>()), pointcloud);

      lidar_array.addPointcloud(m.getTopic(), pointcloud, scan_config);

      if (lidar_array.hasAtleastNScans(use_n_scans)) {
        break;
      }
    } else if (m.getDataType() == std::string("geometry_msgs/TwistStamped")) {
      geometry_msgs::TwistStamped twist_msg =
          *(m.instantiate<geometry_msgs::TwistStamped>());
      odom.addRawOdomData((twist_msg.header.stamp.toSec() * 1000000),
                          twist_msg.twist.linear.x, twist_msg.twist.angular.z);
    } else if (m.getDataType() ==
               std::string("geometry_msgs/TransformStamped")) {
      geometry_msgs::TransformStamped transform_msg =
          *(m.instantiate<geometry_msgs::TransformStamped>());

      kindr::minimal::QuatTransformation T;
      tf::transformMsgToKindr(transform_msg.transform, &T);
      odom.addTransformData(transform_msg.header.stamp.toSec() * 1000000,
                            T.cast<Scalar>());
    }
  }

  //table_ptr->updateHeader("Interpolating odometry data");
  std::vector<Lidar>& lidar_vector = lidar_array.getLidarVector();
  for (Lidar& lidar : lidar_vector) {
    lidar.setOdomOdomTransforms(odom);
  }

  Aligner::Config aligner_config;
  int knn_batch_size_int = aligner_config.knn_batch_size;
  nh_private.param("knn_batch_size", knn_batch_size_int, knn_batch_size_int);
  aligner_config.knn_batch_size = knn_batch_size_int;
  int knn_k_int = aligner_config.knn_k;
  nh_private.param("knn_k", knn_k_int, knn_k_int);
  aligner_config.knn_k = knn_k_int;
  nh_private.param("knn_max_dist", aligner_config.knn_max_dist,
                   aligner_config.knn_max_dist);
  nh_private.param("joint_self_compare", aligner_config.joint_self_compare,
                   aligner_config.joint_self_compare);

  Aligner aligner(table_ptr, aligner_config);

  //table_ptr->updateHeader("Finding individual odometry-lidar transforms");
  for (Lidar& lidar : lidar_vector) {
    //table_ptr->updateHeader(
    //    "Finding individual odometry-lidar transforms: (roll, pitch, yaw)");
    aligner.lidarOdomTransform(3, &lidar);
    //table_ptr->updateHeader(
    //    "Finding individual odometry-lidar transforms: (x, y, roll, pitch, "
    //    "yaw)");
    aligner.lidarOdomTransform(5, &lidar);
  }

  table_ptr->updateHeader(
      "Finding joint odometry-lidar transforms: (x, y, z, roll, pitch, yaw)");
  aligner.lidarOdomJointTransform(6, &lidar_array);

  for (Lidar& lidar : lidar_vector) {
    std::string s = lidar.getId();
    std::replace(s.begin(), s.end(), '/', '_');
    lidar.saveCombinedPointcloud("/home/z/Desktop/" + s + ".ply");
  }

  table_ptr->updateHeader("Saving data");
  rosbag::Bag bag_out;
  bag_out.open("/home/z/datasets/kitti/out.bag", rosbag::bagmode::Write);

  size_t odom_idx = 0;
  for (const rosbag::MessageInstance& m : view) {
    if (!ros::ok()) {
      break;
    }

    if (m.getDataType() == std::string("sensor_msgs/PointCloud2")) {
      Pointcloud pointcloud;
      pcl::fromROSMsg(*(m.instantiate<sensor_msgs::PointCloud2>()), pointcloud);

      pointcloud.header.frame_id += std::string("_out");
      uint sec = pointcloud.header.stamp / 1000000;
      uint nsec = 1000 * (pointcloud.header.stamp - 1000000 * sec);
      ros::Time timestamp(sec, nsec);
      bag_out.write(m.getTopic() + std::string("_out"), timestamp, pointcloud);

      geometry_msgs::TransformStamped transform_msg;
      transform_msg.header.frame_id = "odom";
      transform_msg.header.stamp = timestamp;
      transform_msg.child_frame_id =
          (m.instantiate<sensor_msgs::PointCloud2>())->header.frame_id +
          std::string("_out");

      tf::tfMessage tf_msg;
      tf::transformKindrToMsg(lidar_array.getLidar(m.getTopic())
                                  .getOdomLidarTransform()
                                  .cast<double>(),
                              &transform_msg.transform);
      tf_msg.transforms.push_back(transform_msg);

      geometry_msgs::TransformStamped odom_transform_msg;
      odom_transform_msg.header.frame_id = "world";
      odom_transform_msg.header.stamp = timestamp;
      odom_transform_msg.child_frame_id = "odom";
      tf::transformKindrToMsg(odom.getOdomTransform(timestamp.toSec() * 1000000,
                                                    odom_idx, &odom_idx)
                                  .cast<double>(),
                              &odom_transform_msg.transform);
      tf_msg.transforms.push_back(odom_transform_msg);

      bag_out.write("/tf", timestamp, tf_msg);

      rosgraph_msgs::Clock clock_msg;
      clock_msg.clock = timestamp;
      bag_out.write("/clock", timestamp, clock_msg);
    }
  }

  return 0;
}
