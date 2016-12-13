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

  Scalar lidarOdomProjGridError(const Lidar& lidar, const Scalar& grid_res) const;

  Scalar lidarsOdomProjGridError(Lidars& lidars, const Scalar& grid_res) const;

  void lidarOdomTransform(const size_t num_params, Lidar* lidar_ptr);

  void lidarOdomJointTransform(const size_t num_params, Lidars* lidars_ptr);

  static Transform vecToTransform(const std::vector<double>& vec, const Transform& inital_T);

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

  Config config_;
};

#endif  // LIDAR_ALIGN_ALIGNER_H_