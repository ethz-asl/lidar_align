#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
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
constexpr int kDefaultUseNScans = 100;

// this entire function is an ugly hack that needs deleting
bool topicToLidarId(const std::string& topic_name, LidarId* lidar_id) {

  *lidar_id = topic_name;

  if (topic_name.find("lower") == std::string::npos) {
    return false;
  }/*
  if (topic_name.find("3") != std::string::npos) {
    return false;
  }*/
  if (topic_name.find("5") == std::string::npos) {
    return false;
  }
/*
  static std::map<std::string, size_t> sub_map;
  if (sub_map.count(topic_name) == 0) {
    sub_map[topic_name] = 1;
    return false;
  } else if (sub_map.at(topic_name) > 10) {
    sub_map[topic_name] = 0;
  } else {
    sub_map[topic_name]++;
    return false;
  }
*/
  return true;
}

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

  rosbag::Bag bag;
  bag.open(input_bag_path, rosbag::bagmode::Read);

  std::vector<std::string> types;
  types.push_back(std::string("sensor_msgs/PointCloud2"));
  types.push_back(std::string("geometry_msgs/TwistStamped"));
  rosbag::View view(bag, rosbag::TypeQuery(types));

  std::vector<std::string> column_names;
  column_names.push_back("x");
  column_names.push_back("y");
  column_names.push_back("z");
  column_names.push_back("rx");
  column_names.push_back("ry");
  column_names.push_back("rz");

  std::shared_ptr<Table> table_ptr = std::make_shared<Table>(column_names, 20, 10);
  table_ptr->updateHeader("Loading data");

  LidarArray lidar_array;
  Odom odom;

  for (const rosbag::MessageInstance& m : view) {
    if (m.getDataType() == std::string("sensor_msgs/PointCloud2")) {
      pcl::PointCloud<pcl::PointXYZI> pointcloud;
      pcl::fromROSMsg(*(m.instantiate<sensor_msgs::PointCloud2>()), pointcloud);

      LidarId lidar_id;
      if (topicToLidarId(m.getTopic(), &lidar_id)) {
        lidar_array.addPointcloud(lidar_id, pointcloud);
      }

      if (lidar_array.hasAtleastNScans(use_n_scans)) {
        break;
      }
    } else if (m.getDataType() == std::string("geometry_msgs/TwistStamped")) {
      geometry_msgs::TwistStamped twist_msg =
          *(m.instantiate<geometry_msgs::TwistStamped>());
      odom.addRawOdomData((twist_msg.header.stamp.toSec() * 1000000),
                          twist_msg.twist.linear.x, twist_msg.twist.angular.z);
    }
  }

  table_ptr->updateHeader("Interpolating odometry data");
  std::vector<Lidar>& lidar_vector = lidar_array.getLidarVector();
  for (Lidar& lidar : lidar_vector) {
    lidar.setOdomOdomTransforms(odom);
  }

  Aligner aligner(table_ptr);


  table_ptr->updateHeader("Finding individual odometry-lidar transforms");
  for (Lidar& lidar : lidar_vector) {
    lidar.saveCombinedPointcloud("/home/z/datasets/ibeo/a.ply");
    aligner.lidarOdomTransform(1, &lidar);
    lidar.saveCombinedPointcloud("/home/z/datasets/ibeo/b.ply");
    aligner.lidarOdomTransform(5, &lidar);
  }
  table_ptr->updateHeader("Finding joint odometry-lidar transforms");
  aligner.lidarOdomJointTransform(6, &lidar_array);

  table_ptr->updateHeader("Saving data");
  rosbag::Bag bag_out;
  bag_out.open("/home/z/datasets/ibeo/out.bag", rosbag::bagmode::Write);

  for (const rosbag::MessageInstance& m : view) {
    if (m.getDataType() == std::string("sensor_msgs/PointCloud2")) {
      pcl::PointCloud<pcl::PointXYZI> pointcloud;
      pcl::fromROSMsg(*(m.instantiate<sensor_msgs::PointCloud2>()), pointcloud);

      bag_out.write(m.getTopic(), m.getTime(), pointcloud);

      LidarId lidar_id;
      topicToLidarId(m.getTopic(), &lidar_id);

      geometry_msgs::TransformStamped transform_msg;
      transform_msg.header.frame_id = "odom";
      transform_msg.header.stamp = m.getTime();
      transform_msg.child_frame_id = lidar_id;

      tf::tfMessage tf_msg;
      tf::transformKindrToMsg(
          lidar_array.getLidar(lidar_id).getOdomLidarTransform().cast<double>(),
          &transform_msg.transform);
      tf_msg.transforms.push_back(transform_msg);
      bag_out.write("/tf", m.getTime(), tf_msg);
    }
  }

  return 0;
}
