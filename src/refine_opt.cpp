#include "lidar_align/refine_opt.h"

RefineOpt::RefineOpt(const std::shared_ptr<const LidarAligner> lidar_aligner_ptr
                   size_t max_frames, double min_overlap, double inlier_ratio)
    : lidar_aligner_ptr_(lidar_aligner_ptr),
      max_frames_(max_frames),
      min_overlap_(min_overlap),
      inlier_ratio_(inlier_ratio) {

}

RefineOpt::RawDataToMap(double* raw_data);


std::map<int, double> RoughOpt::Run(std::map<int, kindr::minimal::QuatTransformation> init_T_lidar_odom) {

  std:vector<double> tform_vec;
  lidar_aligner_ptr_->GetVecFromTformMap(tform_vec, true);
  size_t num_params_ = tform_vec.size();

  // Build the problem.
  Problem problem;

  CostFunction* cost_function = new NumericDiffCostFunction<RefineOpt, ceres::CENTRAL, 1, 6>(this);
  problem.AddResidualBlock(this, NULL, tform_vec.data());

  // Run the solver!
  Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  Solver::Summary summary;
  Solve(options, &problem, &summary);
}

bool RefineOpt::operator()(const double* const tform_vec_raw, double* residual){

  std::vector<double> tform_vec;
  tform_vec.assign(tform_vec_raw, tform_vec_raw + num_params_);
  lidar_aligner_ptr_->SetTformMapFromVec(tform_vec, true);
  *residual = lidar_aligne_ptr->getErrorBetweenOverlappingLidars(inlier_ratio_, min_overlap_);

  return true;
}
