#include <ros/ros.h>

#include "lidar_align_vis/lidar_align_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_align_vis");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  lidar_align::LidarAlignServer lidar_align_server(nh, nh_private);

  ros::spin();
  return 0;
}
