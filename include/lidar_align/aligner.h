#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

#include <ncurses.h>
#include <ros/ros.h>
#include <future>
#include <limits>
#include <nlopt.hpp>

#include "lidar_align/sensors.h"
#include "lidar_align/table.h"

namespace lidar_align {

class Aligner {
 public:
  struct Config {
    bool local = true;
    std::vector<double> inital_guess{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> range{1.0, 1.0, 1.0, 3.15, 3.15, 3.15, 0.0};
    double max_evals = 1000;
    double xtol = 0.000001;

    int knn_batch_size = 1000;
    int knn_k = 1;
    float knn_max_dist = 0.1;
    bool time_cal = false;

    std::string output_pointcloud_path = "";
    std::string output_calibration_path = "";
  };

  struct OptData {
    Lidar* lidar;
    Odom* odom;
    Aligner* aligner;
    std::shared_ptr<Table> table;
    bool time_cal;
  };

  Aligner(const std::shared_ptr<Table>& table_ptr, const Config& config);

  static Config getConfig(ros::NodeHandle* nh);

  void lidarOdomTransform(Lidar* lidar, Odom* odom);

 private:
  static Scalar kNNError(
      const pcl::KdTreeFLANN<Point>& kdtree, const Pointcloud& pointcloud,
      const size_t k, const float max_dist, const size_t start_idx = 0,
      const size_t end_idx = std::numeric_limits<size_t>::max());

  Scalar lidarOdomKNNError(const Pointcloud& base_pointcloud,
                           const Pointcloud& combined_pointcloud) const;

  Scalar lidarOdomKNNError(const Lidar& lidar) const;

  static double LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad, void* f_data);

  Config config_;
  std::shared_ptr<Table> table_ptr_;
};

}  // namespace lidar_align

#endif  // LIDAR_ALIGN_ALIGNER_H_
