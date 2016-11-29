#include "lidar_align/t_grid.h"

TGrid::TGrid(size_t num_tforms)
    : consistent_(false),
      num_tforms_(num_tforms),
      T_(num_tforms_ * num_tforms_),
      weights_(num_tforms_ * num_tforms_, 0.0) {}

bool TGrid::set(size_t i, size_t j, const Transform& T, double weight) {
  size_t idx = j + i * num_tforms_;
  if (idx > T_.size()) {
    return false;
  }
  consistent_ = false;
  T_[idx] = T;
  weights_[idx] = weight;
  return true;
}

bool TGrid::get(size_t i, size_t j, Transform* T, double* weight) const {
  size_t idx = j + i * num_tforms_;
  if (idx > T_.size()) {
    return false;
  }
  *T = T_[idx];
  *weight = weights_[idx];
  return true;
}

bool TGrid::isConsistent() const { return consistent_; }

void TGrid::makeConsistent() {
  std::vector<double> tform_vec(6*num_tforms_, 0);

  // Build the problem.
  ceres::Problem problem;

  ceres::CostFunction* cost_function =
      new ceres::NumericDiffCostFunction<TGrid, ceres::CENTRAL, 1, 30>(
          this);
  problem.AddResidualBlock(cost_function, NULL, tform_vec.data());

  // Run the solver!
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // reform the grid
  for (size_t i = 0; i < num_tforms_; ++i) {
    Transform::Vector6 T_O_A;
    for (size_t k = 0; k < T_O_A.size(); ++k) {
      T_O_A[k] = tform_vec[i * T_O_A.size() + k];
    }
    set(i, 0, Transform::exp(T_O_A), 1.0);
  }

  for (size_t i = 0; i < num_tforms_; ++i) {
    for (size_t j = 1; j < num_tforms_; ++j) {
      Transform T_O_A, T_O_B;
      double weight_A, weight_B;
      get(i, 0, &T_O_A, &weight_A);
      get(j, 0, &T_O_B, &weight_B);

      set(i, j, T_O_A.inverse() * T_O_B, 1.0);
    }
  }
}

bool TGrid::operator()(const double* const tform_vec_raw, double* residual) const {
  Transform::Vector6 error_vec;

  for (size_t i = 1; i < num_tforms_; ++i) {
    Transform::Vector6 T_O_A;
    for (size_t k = 0; k < T_O_A.size(); ++k) {
      T_O_A[k] = tform_vec_raw[i * T_O_A.size() + k];
    }
    for (size_t j = 0; j < num_tforms_; ++j) {
      Transform::Vector6 T_O_B;
      for (size_t k = 0; k < T_O_A.size(); ++k) {
        T_O_B[k] = tform_vec_raw[j * T_O_B.size() + k];
      }

      Transform T_B_A_opt =
          (Transform::exp(T_O_A).inverse() * Transform::exp(T_O_B)).inverse();
      Transform T_A_B;
      double weight;
      get(i, j, &T_A_B, &weight);

      error_vec += (T_B_A_opt * T_A_B).log();
    }
  }

  *residual = 0;
  for (size_t k = 0; k < error_vec.size(); ++k) {
    *residual += error_vec[k];
  }
}
