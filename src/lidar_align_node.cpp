#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/tfMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

#include "lidar_align/aligner.h"
#include "lidar_align/sensors.h"

// number of frames to take when calculating rough 2D alignment
constexpr int kDefaultUseNScans = 1000000;

// this entire function is an ugly hack that needs deleting
bool topicToLidarId(const std::string& topic_name, LidarId* lidar_id) {

  std::string topic_start = "lidar_";

  *lidar_id = std::strtol(
      &topic_name[topic_name.find(topic_start) + topic_start.size()], nullptr,
      10);

  /*if (topic_name.find("lower") == std::string::npos) {
    return false;
  }
  if (topic_name.find("3") != std::string::npos) {
    return false;
  }
  if (topic_name.find("4") == std::string::npos) {
    return false;
  }*/

  static std::map<std::string, size_t> sub_map;
  if(sub_map.count(topic_name) == 0){
    sub_map[topic_name] = 1;
    return false;
  }
  else if(sub_map.at(topic_name) > 10){
    sub_map[topic_name] = 0;
  }
  else{
    sub_map[topic_name]++;
    return false;
  }

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

  ROS_INFO("Loading data...");

  Lidars lidars;
  Odom odom;

  for (const rosbag::MessageInstance& m : view) {
    if (m.getDataType() == std::string("sensor_msgs/PointCloud2")) {
      pcl::PointCloud<pcl::PointXYZI> pointcloud;
      pcl::fromROSMsg(*(m.instantiate<sensor_msgs::PointCloud2>()), pointcloud);

      LidarId lidar_id;
      if (topicToLidarId(m.getTopic(), &lidar_id)) {
        lidars.addPointcloud(lidar_id, pointcloud);
      }

      if (lidars.hasAtleastNScans(use_n_scans)) {
        break;
      }
    } else if (m.getDataType() == std::string("geometry_msgs/TwistStamped")) {
      geometry_msgs::TwistStamped twist_msg =
          *(m.instantiate<geometry_msgs::TwistStamped>());
      odom.addRawOdomData((twist_msg.header.stamp.toSec() * 1000000),
                          twist_msg.twist.linear.x, twist_msg.twist.angular.z);
    }
  }
  ROS_INFO("Loading finished");

  ROS_INFO("Interpolating odom data");
  std::vector<Lidar>& lidar_vector = lidars.getLidarsRef();
  for (Lidar& lidar : lidar_vector) {
    lidar.setOdomOdomTransforms(odom);
  }


  Aligner aligner;

  /*std::vector<Lidar> lidar_vector = lidars.getLidarsRef();
  ROS_INFO("Finding lidar-lidar transforms");
  for(Lidar& lidar : lidar_vector){
    ROS_INFO_STREAM("Setting transforms for lidar " << lidar.getId());
    aligner.setLidarTransforms(&lidar);
  }*/

  ROS_INFO("Finding odom-lidar transforms");
  for (Lidar& lidar : lidar_vector) {
    ROS_INFO_STREAM("Setting transforms for lidar " << lidar.getId());
    lidar.saveCombinedPointcloud("/home/z/datasets/ibeo/a.ply");
    aligner.lidarOdomTransform(1, &lidar);
    lidar.saveCombinedPointcloud("/home/z/datasets/ibeo/b.ply");
    aligner.lidarOdomTransform(5, &lidar);
    lidar.saveCombinedPointcloud("/home/z/datasets/ibeo/c.ply");
  }
  //aligner.lidarOdomJointTransform(2, &lidars);
  aligner.lidarOdomJointTransform(6, &lidars);

  ROS_INFO("Saving data");
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
      transform_msg.child_frame_id = std::string("lidar_") + std::to_string(lidar_id);

      tf::tfMessage tf_msg;
      tf::transformKindrToMsg(lidars.getLidar(lidar_id).getOdomLidarTransform(), &transform_msg.transform);
      tf_msg.transforms.push_back(transform_msg);
      bag_out.write("/tf", m.getTime(), tf_msg);

    }
  }

  return 0;
}
