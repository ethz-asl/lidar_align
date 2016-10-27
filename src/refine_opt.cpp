#include "lidar_align/refine_opt.h"

RefineOpt::RefineOpt(const std::shared_ptr<const LidarAligner> lidar_aligner_ptr
                   size_t max_frames, double min_overlap, double overlap_saftey_factor)
    : lidar_aligner_ptr_(lidar_aligner_ptr),
      max_frames_(max_frames),
      min_overlap_(min_overlap),
      overlap_saftey_factor_(overlap_saftey_factor) {

  }


std::map<int, double> RoughOpt::Run(std::map<int, kindr::minimal::QuatTransformation> init_T_lidar_odom) {
  std::vector<int> lidar_ids = lidar_aligner_ptr_->getLidarIds();

  //construct inital guess (ignore 1st lidar tform to keep things observable)
  std::vector<double> init_T_
  for(std::vector<int>::iterator it = (lidar_ids + 1); it != it.end(); ++it){
    kindr::minimal::QuatTransformation::Vector6 vec = init_T_lidar_odom[it].log();
    for(size_t i = 0; i < vec.size(); ++i){
      init_T_.push_back(vec[i]);
    }
  }


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