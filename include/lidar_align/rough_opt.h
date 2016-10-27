#ifndef ROUGH_OPT_H
#define ROUGH_OPT_H

#include "lidar_align/lidar_aligner.h"

class RoughOpt {
 public:
  RoughOpt(const std::shared_ptr<const LidarAligner> lidar_aligner_ptr,
           size_t max_frames, double inlier_ratio);

  std::map<int, double> Run();

 private:
  size_t max_frames_;
  double inlier_ratio_;

  static constexpr size_t iterations_ = 10;

  const std::shared_ptr<const LidarAligner> lidar_aligner_ptr_;
};

#endif //ROUGH_OPT_H