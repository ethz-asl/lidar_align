#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include "lidar_align/sensors.h"

#include "lidar_align_vis/CalibConfig.h"

namespace lidar_align {

class LidarAlignServer {
 public:
  LidarAlignServer(const ros::NodeHandle& nh,
                   const ros::NodeHandle& nh_private);

  // The main calculation and display function
  void backprojectAndPublish();

  // The reconfigure callback
  void callback(lidar_align_vis::CalibConfig& config, uint32_t level);

 private:
  void advertiseTopics();
  void setupReconfigure();

  // Loads the data
  void loadData();
  void loadInitialTransform();
  void loadCalibration();

  // Publishes the cloud
  void publishPointcloud(Pointcloud* pointcloud);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::Publisher pointcloud_pub_;

  dynamic_reconfigure::Server<lidar_align_vis::CalibConfig> server_;

  // The lidar and odom data
  Lidar lidar_;
  Odom odom_;

  // The current and initial transform
  Transform T_o_l_;
  Transform T_o_l_init_;
  // Time offset
  double time_offset_;
  double time_offset_init_;

  // Frame to publish the pointlcoud in
  std::string frame_id_;
};

}  // namespace lidar_align

#endif  // LIDAR_ALIGN_ALIGNER_H_
