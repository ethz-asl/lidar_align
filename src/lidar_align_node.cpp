#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <geometry_msgs/TwistStamped.h>

#include "lidar_align/aligner.h"
#include "lidar_align/sensors.h"

// number of frames to take when calculating rough 2D alignment
constexpr int kDefaultUseNScans = 100;

// this entire function is an ugly hack that needs deleting
bool topicToLidarId(const std::string& topic_name, LidarId* lidar_id) {
  if (topic_name.find("lower") == std::string::npos) {
    return false;
  }

  std::string topic_start = "lidar_";

  *lidar_id = std::strtol(
      &topic_name[topic_name.find(topic_start) + topic_start.size()], nullptr,
      10);
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
  std::vector<Lidar> lidar_vector = lidars.getLidarsRef();
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
    std::cerr << aligner.lidarOdomCPError(lidar) << std::endl;
    aligner.lidarOdomCPTransform(&lidar);
    break;
  }

  return 0;
}
