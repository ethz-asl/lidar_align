#ifndef LIDAR_ALIGN_LOADER_H_
#define LIDAR_ALIGN_LOADER_H_

#include <ros/ros.h>

#include "lidar_align/sensors.h"
#include "lidar_align/table.h"

namespace lidar_align {

class Loader {
 public:
  struct Config {
    int use_n_scans = std::numeric_limits<int>::max();
  };

  Loader(const std::shared_ptr<Table>& table_ptr, const Config& config);

  bool loadPointcloudFromROSBag(const std::string& bag_path,
                                const Scan::Config& scan_config, Lidar* lidar);

  bool loadTformFromROSBag(const std::string& bag_path, Odom* odom);

  bool loadTformFromMaplabCSV(const std::string& csv_path, Odom* odom);

  static Config getConfig(ros::NodeHandle* nh);

 private:
  static bool getNextCSVTransform(std::istream& str, Timestamp* stamp,
                                  kindr::minimal::Position* pos,
                                  kindr::minimal::RotationQuaternion* rot);

  Config config_;
  std::shared_ptr<Table> table_ptr_;
};

}  // namespace lidar_align

#endif  // LIDAR_ALIGN_ALIGNER_H_
