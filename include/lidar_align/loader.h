#ifndef LIDAR_ALIGN_LOADER_H_
#define LIDAR_ALIGN_LOADER_H_

#include "lidar_align/sensors.h"
#include "lidar_align/table.h"

class Loader {
 public:
  struct Config {
    // set default values
    Config() { use_n_scans = std::numeric_limits<int>::max(); }

    int use_n_scans;
  };

  Loader(const std::shared_ptr<Table>& table_ptr,
         const Config& config = Config());

  bool loadPointcloudFromROSBag(const std::string& bag_path,
                                const Scan::Config& scan_config,
                                LidarArray* lidar_array);

  bool loadTformFromROSBag(const std::string& bag_path, Odom* odom);

  bool loadTformFromMaplabCSV(const std::string& csv_path, Odom* odom);

 private:
  static bool getNextCSVTransform(std::istream& str, Timestamp* stamp,
                                  kindr::minimal::Position* pos,
                                  kindr::minimal::RotationQuaternion* rot);

  Config config_;
  std::shared_ptr<Table> table_ptr_;
};

#endif  // LIDAR_ALIGN_ALIGNER_H_