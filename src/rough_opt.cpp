#include "lidar_align/rough_opt.h"

RoughOpt::RoughOpt(const std::shared_ptr<const LidarAligner> lidar_aligner_ptr,
                   size_t max_frames, double inlier_ratio)
    : lidar_aligner_ptr_(lidar_aligner_ptr),
      max_frames_(max_frames),
      inlier_ratio_(inlier_ratio) {}

std::map<int, double> RoughOpt::Run() {
  std::vector<int> lidar_ids = lidar_aligner_ptr_->getLidarIds();

  std::map<int, double> results;

  for (int lidar_id : lidar_ids) {

    std::vector<double> x, y;
    
    size_t use_n_frames = std::min(
        max_frames_, lidar_aligner_ptr_->getNumFrames(lidar_id) - 1);
    for (size_t i = 0; i < use_n_frames; ++i) {
      kindr::minimal::QuatTransformation T;

      T = lidar_aligner_ptr_->getICPTransformBetweenTimesteps(T, lidar_id, i, inlier_ratio_, iterations_);
      x.push_back(T.getPosition()(0));
      y.push_back(T.getPosition()(1));
      //ROS_ERROR_STREAM("Lidar: " << lidar_id << " scan: " << i << " tform: " << T.log());
    }

    std::sort(x.begin(), x.end());
    double median_x = x[x.size()/2];
    std::sort(y.begin(), y.end());
    double median_y = y[y.size()/2];

    double angle = std::atan2(median_y, median_x);
    results[lidar_id] = angle;
    ROS_ERROR_STREAM("Lidar: " << lidar_id << " angle: " << angle);

  }
}