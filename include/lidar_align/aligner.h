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

  Scalar lidarOdomProjGridError(const Lidar& lidar) const;

  void lidarOdomTransform(Lidar* lidar_ptr);

  static Transform rawVec6ToTransform(const double* vec6);

  static Transform rawVec5ToTransform(const double* vec5);

  static Transform rawVec3ToTransform(const double* vec3);

  static Transform vecToTransform(const std::vector<double>& vec);

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

  static double LidarOdomMinimizer(const std::vector<double> &x, std::vector<double> &grad, void* f_data);

  Scalar trimmedMeans(const std::vector<Scalar>& raw_error,
                      const Scalar& inlier_ratio) const;

  Config config_;
};

#endif  // LIDAR_ALIGN_ALIGNER_H_