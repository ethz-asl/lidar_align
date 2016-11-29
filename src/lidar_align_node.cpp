#include "lidar_align/lidar_aligner.h"
#include "lidar_align/rough_opt.h"
#include "lidar_align/t_grid.h"

// number of frames to take when calculating rough 2D alignment
static constexpr int kDefaultUseNScans = 1000;

// place sensors this distance from origin as inital guess
static constexpr double kDefaultInitalLidarDistFromOrigin = 0;

static constexpr double kDefaultMinOverlap = 0.4;

static constexpr double kDefaultOverlapSafteyMargin = 0.5;

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

    if (lidar_aligner_ptr->hasAtleastNScans(use_n_scans)) {
      break;
    }
  }

  ROS_INFO("Finding inital alignment guess...");

  // get angles
  RoughOpt rough_opt(lidar_aligner_ptr);
  std::map<LidarAligner::LidarId, double> rough_angles = rough_opt.Run();

  // build inital guess from angle
  double inital_lidar_dist_from_origin;
  nh_private.param("inital_lidar_dist_from_origin",
                   inital_lidar_dist_from_origin,
                   kDefaultInitalLidarDistFromOrigin);
  for (const std::pair<LidarAligner::LidarId, double>& angle : rough_angles) {

    // creates a tform rotated by angle found in the rough alignment stage and a
    // fixed distance from the origin in the direction the sensor is facing
    kindr::minimal::QuatTransformation T_l_o(
        kindr::minimal::RotationQuaternion(
            kindr::minimal::AngleAxis(angle.second, 0.0, 0.0, 1.0)),
        kindr::minimal::Position(
            inital_lidar_dist_from_origin * std::cos(angle.second),
            inital_lidar_dist_from_origin * std::sin(angle.second), 0));

    lidar_aligner_ptr->setTform(T_l_o, angle.first);
  }

  ROS_INFO("Refining alignment...");

  double min_overlap;
  nh_private.param("min_overlap", min_overlap, kDefaultMinOverlap);
  double overlap_saftey_margin;
  nh_private.param("overlap_saftey_margin", overlap_saftey_margin,
                   kDefaultOverlapSafteyMargin);

  std::vector<LidarAligner::LidarId> lidar_ids =
      lidar_aligner_ptr->getLidarIds();

  TGrid T_grid(lidar_ids.size());

  for (size_t i = 0; i < lidar_ids.size(); ++i) {
    for (size_t j = i; j < lidar_ids.size(); ++j) {
      if (i == j) {
        continue;
      }

      float overlap = lidar_aligner_ptr->getSensorOverlap(lidar_ids[i], lidar_ids[j]);
      ROS_ERROR_STREAM("overlap: " << overlap);
      if (overlap > min_overlap) {
        LidarAligner::Transform T_A_B = lidar_aligner_ptr->getICPTransformBetweenLidars(
            lidar_ids[i], lidar_ids[j], overlap_saftey_margin, 20);
        ROS_ERROR_STREAM(" " << i << " " << j << "\n" << T_A_B);
        T_grid.set(i, j, T_A_B, overlap - overlap_saftey_margin);
      }
    }
  }

  //T_grid.makeConsistent();

  // lidar_aligner_ptr->

  return 0;
}
