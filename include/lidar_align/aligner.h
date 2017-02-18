#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

#include <ncurses.h>
#include <future>
#include <limits>
#include <nlopt.hpp>

#include "lidar_align/sensors.h"
#include "lidar_align/table.h"

class Aligner {
 public:
  struct Config {
    // set default values
    Config() {
      knn_batch_size = 5000;
      knn_k = 5;
      knn_max_dist = 0.1;
    }

    size_t knn_batch_size;
    size_t knn_k;
    float knn_max_dist;
    bool joint_self_compare;
  };

  Aligner(const std::shared_ptr<Table>& table_ptr,
          const Config& config = Config());

  void lidarOdomTransform(const size_t num_params, Lidar* lidar_ptr);

  void lidarOdomJointTransform(const size_t num_params,
                               LidarArray* lidar_array_ptr);

 private:
  void updateTableRow(const Lidar& lidar);

  void updateTableFooter(const Scalar error);

  static Scalar kNNError(
      const pcl::KdTreeFLANN<PointN>& kdtree, const PointcloudN& pointcloud,
      const size_t k, const float max_dist, const size_t start_idx = 0,
      const size_t end_idx = std::numeric_limits<size_t>::max());

  Scalar lidarOdomKNNError(const PointcloudN& base_pointcloud,
                           const PointcloudN& combined_pointcloud) const;

  Scalar lidarOdomKNNError(const Lidar& lidar) const;

  Scalar lidarOdomKNNError(const LidarArray& lidar_array) const;

  static double LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad, void* f_data);

  static double LidarOdomJointMinimizer(const std::vector<double>& x,
                                        std::vector<double>& grad,
                                        void* f_data);

  static Transform vecToTransform(const std::vector<double>& vec,
                                  const Transform& inital_T);

  static std::vector<double> transformToVec(const Transform& T,
                                            size_t vec_length = 6);

  static std::vector<double> createRangeVec(const std::vector<double>& full_vec,
                                            size_t vec_length = 6);

  Config config_;
  std::shared_ptr<Table> table_ptr_;
};

#endif  // LIDAR_ALIGN_ALIGNER_H_