#ifndef ROUGH_OPT_H
#define ROUGH_OPT_H

#include "lidar_align/lidar_aligner.h"

class RoughOpt {
 public:
  RoughOpt(const std::shared_ptr<const LidarAligner> lidar_aligner_ptr);

  std::map<LidarAligner::LidarId, double> Run();

 private:
  const std::shared_ptr<const LidarAligner> lidar_aligner_ptr_;
};

#endif //ROUGH_OPT_H