#include <ros/ros.h>

#include <algorithm>

#include "lidar_align/aligner.h"
#include "lidar_align/loader.h"
#include "lidar_align/sensors.h"
#include "lidar_align/table.h"

using namespace lidar_align;

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

  Loader loader(table_ptr, Loader::getConfig(&nh_private));

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
                                       Scan::getConfig(&nh_private), &lidar) ||
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

  Aligner aligner(table_ptr, Aligner::getConfig(&nh_private));

  aligner.lidarOdomTransform(&lidar, &odom);

  return 0;
}
