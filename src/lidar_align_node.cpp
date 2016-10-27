#include "lidar_align/lidar_aligner.h"
#include "lidar_align/rough_opt.h"

// number of frames to take when calculating rough 2D alignment
static constexpr int kDefaultUseNFramesRough = 100;

// number of frames to take when calculating fine 3D alignment
static constexpr int kDefaultUseNFramesFine = 10;

static constexpr double kDefaultSuccessiveInlierRatio = 0.95;

// place sensors this distance from origin as inital guess
static constexpr double kDefaultInitalLidarDistFromOrigin = 1;

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_align");

  ros::NodeHandle nh, nh_private("~");

  std::string input_bag_path;
  if (!nh_private.getParam("input_bag_path", input_bag_path)) {
    ROS_FATAL("Could not find input_bag_path parameter, exiting");
    exit(EXIT_FAILURE);
  }

  int use_n_frames_rough;
  nh_private.param("use_n_frames_rough", use_n_frames_rough,
                   kDefaultUseNFramesRough);

  int use_n_frames_fine;
  nh_private.param("use_n_frames_fine", use_n_frames_fine,
                   kDefaultUseNFramesFine);

  double successive_inlier_ratio;
  nh_private.param("successive_inlier_ratio", successive_inlier_ratio,
                   kDefaultSuccessiveInlierRatio);

  std::shared_ptr<LidarAligner> lidar_aligner_ptr(
      new LidarAligner(nh, nh_private));

  rosbag::Bag bag;
  bag.open(input_bag_path, rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TypeQuery("sensor_msgs/PointCloud2"));

  ROS_INFO("Loading scans...");

  for (const rosbag::MessageInstance& m : view) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    pcl::fromROSMsg(*(m.instantiate<sensor_msgs::PointCloud2>()), pointcloud);

    lidar_aligner_ptr->addLidarScan(pointcloud, m.getTopic());

    if (lidar_aligner_ptr->hasAtleastNScans(
            std::max(use_n_frames_fine, use_n_frames_rough))) {
      break;
    }
  }

  ROS_INFO("Finding inital alignment guess...");

  // get angles
  RoughOpt rough_opt(lidar_aligner_ptr, use_n_frames_rough,
                     successive_inlier_ratio);
  std::map<int, double> rough_angles = rough_opt.Run();

  // build inital guess from angle
  double inital_lidar_dist_from_origin;
  nh_private.param("inital_lidar_dist_from_origin",
                   inital_lidar_dist_from_origin,
                   kDefaultInitalLidarDistFromOrigin);
  std::map<int, kindr::minimal::QuatTransformation> T_lidars_odom;
  for (const std::pair<int, double>& angle : rough_angles) {
    // creates a tform rotated by angle found in the rough alignment stage and a
    // fixed distance from the origin in the direction the sensor is facing
    kindr::minimal::QuatTransformation T_lidar_odom(
        kindr::minimal::RotationQuaternion(
            kindr::minimal::AngleAxis(angle.second, 0, 0, 1)),
        kindr::minimal::Position(
            inital_lidar_dist_from_origin * std::cos(angle.second),
            inital_lidar_dist_from_origin * std::sin(angle.second), 0));

    T_lidars_odom[angle.first] = T_lidar_odom;
  }

  ROS_INFO("Refining alignment...");

  return 0;
}
