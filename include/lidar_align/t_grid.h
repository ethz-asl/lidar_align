#ifndef LIDAR_ALIGN_T_GRID_H_
#define LIDAR_ALIGN_T_GRID_H_

#include <kindr/minimal/quat-transformation.h>
#include <ceres/ceres.h>

class TGrid {
 public:

  typedef kindr::minimal::QuatTransformationTemplate<double> Transform;
  typedef kindr::minimal::RotationQuaternionTemplate<double> Rotation;

  TGrid(size_t num_tforms);

  bool set(size_t i, size_t j, const Transform& T, double weight);

  bool get(size_t i, size_t j, Transform* T, double* weight) const;

  bool isConsistent() const;

  void makeConsistent();

  bool operator()(const double* const tform_vec_raw, double* residual) const;

 private:
  bool consistent_;
  const size_t num_tforms_;
  std::vector<Transform> T_;
  std::vector<double> weights_;
};

#endif