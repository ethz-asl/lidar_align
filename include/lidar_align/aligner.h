#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

#include <ceres/ceres.h>
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

  void lidarOdomCPTransform(Lidar* lidar_ptr);

  void lidarOdomProjGridTransform(Lidar* lidar_ptr);

  static Transform rawVec6ToTransform(const double* vec6);

  static Transform rawVec5ToTransform(const double* vec5);

  static Transform rawVec3ToTransform(const double* vec3);

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

  class LidarOdomCPMinimizer {
   public:
    LidarOdomCPMinimizer(const Aligner* aligner, Lidar* lidar_ptr);

    bool operator()(const Scalar* const tform_vec_raw, Scalar* residual) const;

   private:
    Lidar* lidar_ptr_;
    const Aligner* aligner_ptr_;
  };

  class LidarOdomProjGridMinimizer {
   public:
    LidarOdomProjGridMinimizer(const Aligner* aligner, Lidar* lidar_ptr);

    bool operator()(const Scalar* const tform_vec_raw, Scalar* residual) const;

   private:
    Lidar* lidar_ptr_;
    const Aligner* aligner_ptr_;
  };

  Scalar trimmedMeans(const std::vector<Scalar>& raw_error,
                      const Scalar& inlier_ratio) const;

  Config config_;
};

#endif  // LIDAR_ALIGN_ALIGNER_H_