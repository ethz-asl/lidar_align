#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

#include <ncurses.h>
#include <future>
#include <limits>
#include <nlopt.hpp>
#include <ros/ros.h>

#include "lidar_align/sensors.h"
#include "lidar_align/table.h"

class Aligner {
 public:
  struct Config {
    // set default values
    Config() {
      local = true;
      inital_guess = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      range = {1.0, 1.0, 1.0, 3.15, 3.15, 3.15, 0.0};

      max_evals = 1000;
      xtol = 0.000001;

      knn_batch_size = 1000;
      knn_k = 1;
      knn_max_dist = 0.1;
      time_cal = false;

      output_pointcloud_path = "";
      output_calibration_path = "";
    }

    bool local;
    std::vector<double> inital_guess;
    std::vector<double> range;
    double max_evals;
    double xtol;

    int knn_batch_size;
    int knn_k;
    float knn_max_dist;
    bool time_cal;

    std::string output_pointcloud_path;
    std::string output_calibration_path;
  };

  struct OptData {
    Lidar* lidar;
    Odom* odom;
    Aligner* aligner;
    std::shared_ptr<Table> table;
    bool time_cal;
  };

  Aligner(const std::shared_ptr<Table>& table_ptr,
          const Config& config = Config());

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

#endif  // LIDAR_ALIGN_ALIGNER_H_