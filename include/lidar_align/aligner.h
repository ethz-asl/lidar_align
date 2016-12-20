#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

#include <nlopt.hpp>
#include "lidar_align/sensors.h"

class Aligner {
 public:
  void setLidarTransforms(Lidar* lidar);

  Transform scanScanICPTransform(
      const Scan& scan_a, const Scan& scan_b,
      const Transform& T_a_b_inital = Transform()) const;

  Scalar scanScanCPError(const Scan& scan_a, const Scan& scan_b,
                         const Transform& T_a_b) const;

  Scalar lidarOdomCPError(const Lidar& lidar) const;

  Scalar lidarOdomVoxelEntropyError(Lidar& lidar) const;

  Scalar lidarsOdomVoxelEntropyError(Lidars& lidars) const;

  void lidarOdomTransform(const size_t num_params, Lidar* lidar_ptr);

  void lidarOdomJointTransform(const size_t num_params, Lidars* lidars_ptr);

  Scalar lidarOdomKNNError(const Lidar& lidar, const size_t k) const;

  Scalar lidarsOdomKNNError(Lidars& lidars, const size_t k);

  static Transform vecToTransform(const std::vector<double>& vec,
                                  const Transform& inital_T);

  static std::vector<double> transformToVec(const Transform& T,
                                            size_t vec_length = 6);

 private:
  struct Config {
    // set default values
    Config() {
      icp_iterations = 30;
      icp_inlier_ratio = 0.8;
      cp_inlier_ratio = 0.8;
      lidar_odom_cp_inlier_ratio = 0.8;
    };

    size_t icp_iterations;

    Scalar icp_inlier_ratio;
    Scalar cp_inlier_ratio;
    Scalar lidar_odom_cp_inlier_ratio;
  };

  static double LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad, void* f_data);

  static double LidarOdomJointMinimizer(const std::vector<double>& x,
                                        std::vector<double>& grad,
                                        void* f_data);

  Scalar trimmedMeans(const std::vector<Scalar>& raw_error,
                      const Scalar& inlier_ratio) const;

  Scalar thresholdedSum(const std::vector<Scalar>& raw_error,
                                 const Scalar& threshold) const;

  Config config_;
};

class VoxelPyramid {
 public:
  VoxelPyramid(float element_size, size_t levels);

  void addPointcloud(const Pointcloud& pointcloud);

  void addLidar(Lidar& lidar);

  void addLidars(Lidars& lidars);

  Scalar calculateEntropy();

  void clear();

 private:
  float element_size_;
  std::vector<std::map<std::tuple<int, int, int>, int>> voxel_pyramid_;
};

#endif  // LIDAR_ALIGN_ALIGNER_H_