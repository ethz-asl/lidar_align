#include "lidar_align/rough_opt.h"

RoughOpt::RoughOpt(const std::shared_ptr<const LidarAligner> lidar_aligner_ptr)
    : lidar_aligner_ptr_(lidar_aligner_ptr) {}

std::map<LidarAligner::LidarId, double> RoughOpt::Run() {
  std::vector<LidarAligner::LidarId> lidar_ids = lidar_aligner_ptr_->getLidarIds();

  std::map<LidarAligner::LidarId, double> results;

  for (LidarAligner::LidarId lidar_id : lidar_ids) {

    std::vector<double> x, y;
    
    for (size_t i = 0; i < lidar_aligner_ptr_->getNumScans(lidar_id); ++i) {
      kindr::minimal::QuatTransformation T;

      if(lidar_aligner_ptr_->getTransformAtAtp1(lidar_id, i, &T)){
        x.push_back(T.getPosition()(0));
        y.push_back(T.getPosition()(1));
      }
    }

    std::sort(x.begin(), x.end());
    double median_x = x[x.size()/2];
    std::sort(y.begin(), y.end());
    double median_y = y[y.size()/2];

    double angle = std::atan2(median_y, median_x);
    results[lidar_id] = angle;
    ROS_INFO_STREAM("Lidar: " << lidar_id << " angle: " << angle);
  }
    return results;
}