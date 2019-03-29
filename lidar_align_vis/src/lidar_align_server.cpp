#include "lidar_align_vis/lidar_align_server.h"

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <lidar_align/loader.h>
#include <lidar_align/transform.h>

#include "lidar_align_vis/calib_loader.h"

namespace lidar_align {

LidarAlignServer::LidarAlignServer(const ros::NodeHandle& nh,
                                   const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), frame_id_("world"), time_offset_(0.0) {
  // Initial ROS interaction
  advertiseTopics();
  // Getting the data to work with
  loadData();
  loadCalibration();
  setupReconfigure();
}

void LidarAlignServer::advertiseTopics() {
  pointcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "pointcloud", 1, true);
}

void LidarAlignServer::setupReconfigure() {
  dynamic_reconfigure::Server<lidar_align_vis::CalibConfig>::CallbackType f;
  f = boost::bind(&LidarAlignServer::callback, this, _1, _2);
  server_.setCallback(f);
}

void LidarAlignServer::loadData() {
  Loader loader(Loader::getConfig(&nh_private_));

  std::string input_bag_path;
  ROS_INFO("Loading Pointcloud Data...");
  if (!nh_private_.getParam("input_bag_path", input_bag_path)) {
    ROS_FATAL("Could not find input_bag_path parameter, exiting");
    exit(EXIT_FAILURE);
  } else if (!loader.loadPointcloudFromROSBag(
                 input_bag_path, Scan::getConfig(&nh_private_), &lidar_)) {
    ROS_FATAL("Error loading pointclouds from ROS bag.");
    exit(0);
  }

  bool transforms_from_csv;
  nh_private_.param("transforms_from_csv", transforms_from_csv, false);
  std::string input_csv_path;
  ROS_INFO("Loading Transformation Data...                                ");
  if (transforms_from_csv) {
    if (!nh_private_.getParam("input_csv_path", input_csv_path)) {
      ROS_FATAL("Could not find input_csv_path parameter, exiting");
      exit(EXIT_FAILURE);
    } else if (!loader.loadTformFromMaplabCSV(input_csv_path, &odom_)) {
      ROS_FATAL("Error loading transforms from CSV.");
      exit(0);
    }
  } else if (!loader.loadTformFromROSBag(input_bag_path, &odom_)) {
    ROS_FATAL("Error loading transforms from ROS bag.");
    exit(0);
  }

  if (lidar_.getNumberOfScans() == 0) {
    ROS_FATAL("No data loaded, exiting");
    exit(0);
  }

  ROS_INFO("Interpolating Transformation Data...                          ");
  lidar_.setOdomOdomTransforms(odom_);
}

void LidarAlignServer::loadCalibration() {
  std::string initial_calibration_path;
  if (nh_private_.getParam("initial_calibration_path", initial_calibration_path)) {
    ROS_INFO_STREAM("Loading calibration from file:"
                    << std::endl
                    << initial_calibration_path);
    CalibLoader calib_loader;
    calib_loader.loadFromTxt(initial_calibration_path, &T_o_l_init_,
                             &time_offset_init_);
  }
}

void LidarAlignServer::backprojectAndPublish() {
  // Getting the back projected pointcloud
  ROS_INFO("Backprojecting");
  lidar_.setOdomOdomTransforms(odom_, time_offset_);
  lidar_.setOdomLidarTransform(T_o_l_);
  Pointcloud pointcloud;
  lidar_.getCombinedPointcloud(&pointcloud);

  // Publishing the cloud
  ROS_INFO("Publishing");
  publishPointcloud(&pointcloud);
}

void LidarAlignServer::callback(lidar_align_vis::CalibConfig& config,
                                uint32_t level) {
  // Checking for the startup call or reset
  if ((level == std::numeric_limits<uint32_t>::max()) ||
      (config.reset_to_init == true)) {
    config.dtx = 0.0;
    config.dty = 0.0;
    config.dtz = 0.0;
    config.drx = 0.0;
    config.dry = 0.0;
    config.drz = 0.0;
    config.d_time_offset = 0.0;
    config.reset_to_init = false;
  }
  // Using the values
  ROS_INFO_STREAM("Reconfigure Request: ["
                  << config.dtx << ", " << config.dty << ", " << config.dtz
                  << ", " << config.drx << ", " << config.dry << ", "
                  << config.drz << "]");
  const Transform::Vector6 T_log =
      (Transform::Vector6() << static_cast<float>(config.dtx),
       static_cast<float>(config.dty), static_cast<float>(config.dtz),
       static_cast<float>(config.drx), static_cast<float>(config.dry),
       static_cast<float>(config.drz))
          .finished();
  const Transform T_diff = Transform::exp(T_log);
  T_o_l_ = T_diff * T_o_l_init_;
  time_offset_ = time_offset_init_ + config.d_time_offset;
  // Use the updated transform
  backprojectAndPublish();
}

void LidarAlignServer::publishPointcloud(Pointcloud* pointcloud) {
  // Attaching world frame and out it goes
  pointcloud->header.frame_id = frame_id_;
  pointcloud_pub_.publish(*pointcloud);
}

}  // namespace lidar_align