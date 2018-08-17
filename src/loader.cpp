#include <geometry_msgs/TransformStamped.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

#include "lidar_align/loader.h"

Loader::Loader(const std::shared_ptr<Table>& table_ptr, const Config& config)
    : table_ptr_(table_ptr), config_(config) {}

bool Loader::loadPointcloudFromROSBag(const std::string& bag_path,
                                      const Scan::Config& scan_config,
                                      LidarArray* lidar_array) {
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
    return false;
  }

  std::vector<std::string> types;
  types.push_back(std::string("sensor_msgs/PointCloud2"));
  rosbag::View view(bag, rosbag::TypeQuery(types));

  size_t scan_num = 0;
  for (const rosbag::MessageInstance& m : view) {
    std::stringstream ss;
    ss << "Loading scan: " << scan_num++ << " from ros bag";
    table_ptr_->updateHeader(ss.str());

    Pointcloud pointcloud;
    pcl::fromROSMsg(*(m.instantiate<sensor_msgs::PointCloud2>()), pointcloud);

    lidar_array->addPointcloud(m.getTopic(), pointcloud, scan_config);

    if (lidar_array->hasAtleastNScans(config_.use_n_scans)) {
      break;
    }
  }

  return true;
}

bool Loader::loadTformFromROSBag(const std::string& bag_path, Odom* odom) {
  rosbag::Bag bag;
  try {
    bag.open(bag_path, rosbag::bagmode::Read);
  } catch (rosbag::BagException e) {
    ROS_ERROR_STREAM("LOADING BAG FAILED: " << e.what());
  }

  std::vector<std::string> types;
  types.push_back(std::string("geometry_msgs/TransformStamped"));
  rosbag::View view(bag, rosbag::TypeQuery(types));

  size_t tform_num = 0;
  for (const rosbag::MessageInstance& m : view) {
    std::stringstream ss;
    ss << "Loading tform: " << tform_num++ << " from ros bag";
    table_ptr_->updateHeader(ss.str());

    geometry_msgs::TransformStamped transform_msg =
        *(m.instantiate<geometry_msgs::TransformStamped>());

    kindr::minimal::QuatTransformation T;
    tf::transformMsgToKindr(transform_msg.transform, &T);

    Timestamp stamp = transform_msg.header.stamp.sec * 1000000ll +
                      transform_msg.header.stamp.nsec / 1000ll;
    odom->addTransformData(stamp, T.cast<Scalar>());
  }

  return true;
}

bool Loader::loadTformFromMaplabCSV(const std::string& csv_path, Odom* odom) {
  std::ifstream file(csv_path, std::ifstream::in);

  size_t tform_num = 0;
  while (file.peek() != EOF) {
    std::stringstream ss;
    ss << "Loading tform: " << tform_num++ << " from maplab csv";
    table_ptr_->updateHeader(ss.str());

    Timestamp stamp;
    kindr::minimal::Position pos;
    kindr::minimal::RotationQuaternion rot;

    if (getNextCSVTransform(file, &stamp, &pos, &rot)) {
      odom->addTransformData(stamp, kindr::minimal::QuatTransformation(rot, pos).cast<Scalar>());
    }
  }

  return true;
}

// lots of potential failure cases not checked
bool Loader::getNextCSVTransform(std::istream& str, Timestamp* stamp,
                                 kindr::minimal::Position* pos,
                                 kindr::minimal::RotationQuaternion* rot) {
  std::string line;
  std::getline(str, line);

  // ignore comment lines
  if (line[0] == '#') {
    return false;
  }

  std::stringstream line_stream(line);
  std::string cell;

  std::vector<std::string> data;
  while (std::getline(line_stream, cell, ',')) {
    data.push_back(cell);
  }

  if (data.size() < 9) {
    return false;
  }

  constexpr size_t TIME = 0;
  constexpr size_t X = 2;
  constexpr size_t Y = 3;
  constexpr size_t Z = 4;
  constexpr size_t RW = 5;
  constexpr size_t RX = 6;
  constexpr size_t RY = 7;
  constexpr size_t RZ = 8;

  *stamp = std::stoll(data[TIME]) / 1000ll;
  *rot = kindr::minimal::RotationQuaternion(
      std::stod(data[RW]), std::stod(data[RX]), std::stod(data[RY]),
      std::stod(data[RZ]));
  *pos = kindr::minimal::Position(std::stod(data[X]), std::stod(data[Y]),
                                  std::stod(data[Z]));

  return true;
}