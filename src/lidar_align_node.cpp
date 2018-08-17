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

Aligner::Config getAlignerConfig(ros::NodeHandle* nh){
  Aligner::Config config;
  nh->param("local", config.local, config.local);
  nh->param("inital_guess", config.inital_guess, config.inital_guess);
  nh->param("range", config.range, config.range);
  nh->param("max_evals", config.max_evals, config.max_evals);
  nh->param("xtol", config.xtol, config.xtol);
  nh->param("knn_batch_size", config.knn_batch_size, config.knn_batch_size);
  nh->param("knn_k", config.knn_k, config.knn_k);
  nh->param("knn_max_dist", config.knn_max_dist, config.knn_max_dist);

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

  return std::make_shared<Table>(column_names, 20, 10);
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

  LidarArray lidar_array;
  Odom odom;

  if (!loader.loadPointcloudFromROSBag(
          input_bag_path, getScanConfig(&nh_private), &lidar_array) ||
      !loader.loadTformFromMaplabCSV(input_csv_path, &odom)) {
    ROS_FATAL("Data loading failed");
    exit(0);
  }

  if (!lidar_array.hasAtleastNScans(1)) {
    ROS_FATAL("No data loaded, exiting");
    exit(0);
  }

  table_ptr->updateHeader("Interpolating odometry data");
  std::vector<Lidar>& lidar_vector = lidar_array.getLidarVector();
  for (Lidar& lidar : lidar_vector) {
    lidar.setOdomOdomTransforms(odom);
  }

  Aligner aligner(table_ptr, getAlignerConfig(&nh_private));

  table_ptr->updateHeader("Finding individual odometry-lidar transforms");
  for (Lidar& lidar : lidar_vector) {
    table_ptr->updateHeader(
        "Finding odometry-lidar transforms: (x, y, roll, pitch, "
        "yaw)");
    aligner.lidarOdomTransform(6, &lidar);

    std::string s = lidar.getId();
    std::replace(s.begin(), s.end(), '/', '_');
    lidar.saveCombinedPointcloud("/home/z/Desktop/" + s + ".ply");
  }

  for (Lidar& lidar : lidar_vector) {
      ROS_ERROR_STREAM(" " << lidar.getOdomLidarTransform());
  }

  return 0;
}
