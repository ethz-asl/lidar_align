#include <ros/ros.h>

#include <algorithm>

#include <rosgraph_msgs/Clock.h>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>

#include <pcl/common/transforms.h>
#include <pcl_ros/point_cloud.h>

#include "lidar_align/aligner.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"
#include "lidar_align/table.h"

Aligner::Config getAlignerConfig(ros::NodeHandle* nh) {
  Aligner::Config config;
  nh->param("local", config.local, config.local);
  nh->param("inital_guess", config.inital_guess, config.inital_guess);
  nh->param("range", config.range, config.range);
  nh->param("max_evals", config.max_evals, config.max_evals);
  nh->param("xtol", config.xtol, config.xtol);
  nh->param("knn_batch_size", config.knn_batch_size, config.knn_batch_size);
  nh->param("knn_k", config.knn_k, config.knn_k);
  nh->param("knn_max_dist", config.knn_max_dist, config.knn_max_dist);
  nh->param("time_cal", config.time_cal, config.time_cal);

  return config;
}

Scan::Config getScanConfig(ros::NodeHandle* nh) {
  Scan::Config config;
  nh->param("min_point_distance", config.min_point_distance,
            config.min_point_distance);
  nh->param("max_point_distance", config.max_point_distance,
            config.max_point_distance);
  nh->param("keep_points_ratio", config.keep_points_ratio,
            config.keep_points_ratio);
  nh->param("min_return_intensity", config.min_return_intensity,
            config.min_return_intensity);
  return config;
}

Loader::Config getLoaderConfig(ros::NodeHandle* nh) {
  Loader::Config config;
  nh->param("use_n_scans", config.use_n_scans, config.use_n_scans);
  return config;
}

std::shared_ptr<Table> setupTable() {
  std::vector<std::string> column_names;
  column_names.push_back("x");
  column_names.push_back("y");
  column_names.push_back("z");
  column_names.push_back("rx");
  column_names.push_back("ry");
  column_names.push_back("rz");
  column_names.push_back("time");

  return std::make_shared<Table>(column_names, 10, 10);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_align");

  ros::NodeHandle nh, nh_private("~");

  std::shared_ptr<Table> table_ptr = setupTable();

  Loader loader(table_ptr, getLoaderConfig(&nh_private));

  std::string input_bag_path;
  if (!nh_private.getParam("input_bag_path", input_bag_path)) {
    ROS_FATAL("Could not find input_bag_path parameter, exiting");
    exit(EXIT_FAILURE);
  }

  std::string input_csv_path;
  if (!nh_private.getParam("input_csv_path", input_csv_path)) {
    ROS_FATAL("Could not find input_csv_path parameter, exiting");
    exit(EXIT_FAILURE);
  }

  Lidar lidar;
  Odom odom;

  if (!loader.loadPointcloudFromROSBag(input_bag_path,
                                       getScanConfig(&nh_private), &lidar) ||
      !loader.loadTformFromMaplabCSV(input_csv_path, &odom)) {
    ROS_FATAL("Data loading failed");
    exit(0);
  }

  if (lidar.getNumberOfScans() == 0) {
    ROS_FATAL("No data loaded, exiting");
    exit(0);
  }

  table_ptr->updateHeader("Interpolating odometry data");
  lidar.setOdomOdomTransforms(odom);

  Aligner aligner(table_ptr, getAlignerConfig(&nh_private));

  table_ptr->updateHeader("Finding individual odometry-lidar transform");

  aligner.lidarOdomTransform(&lidar, &odom);

  lidar.saveCombinedPointcloud("/home/z/Desktop/lidar.ply");

  ROS_ERROR_STREAM(" " << lidar.getOdomLidarTransform());

  return 0;
}
