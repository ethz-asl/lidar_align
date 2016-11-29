#include <lidar_align/multi_icp.h>
#include <pcl/io/ply_io.h>
#include <iostream>

MultiICP::MultiICP() {}

void MultiICP::addPointCloud(const pcl::PointCloud<pcl::PointXYZI> &src_pc,
                             const pcl::PointCloud<pcl::PointXYZI> &tgt_pc,
                             float inlier_ratio) {
  ICP icp;
  icp.setSrcPoints(src_pc);
  icp.setTgtPoints(tgt_pc);
  icps_.push_back(icp);
  inlier_ratios_.push_back(inlier_ratio);
}

void MultiICP::clearPointClouds() {
  icps_.clear();
  inlier_ratios_.clear();
}

ICP::TransformationD MultiICP::getCurrentTform() const {
  return T_src_tgt_.cast<double>();
}

void MultiICP::setCurrentTform(ICP::Transformation T_src_tgt) {
  T_src_tgt_ = T_src_tgt;
}

void MultiICP::setCurrentTform(ICP::TransformationD T_src_tgt) {
  T_src_tgt_ = T_src_tgt.cast<float>();
}

bool MultiICP::runMultiICP(size_t iterations) {
  for (size_t j = 0; j < iterations; ++j) {
    size_t total_size = 0;
    std::vector<Eigen::Matrix3Xf> src(icps_.size());
    std::vector<Eigen::Matrix3Xf> tgt(icps_.size());

    for (size_t i = 0; i < icps_.size(); ++i) {
      icps_[i].setCurrentTform(T_src_tgt_);

      icps_[i].getFilteredMatchingPoints(inlier_ratios_[i], &src[i], &tgt[i]);

      total_size += src[i].cols();
    }

    Eigen::Matrix3Xf full_src;
    Eigen::Matrix3Xf full_tgt;

    full_src.resize(Eigen::NoChange, total_size);
    full_tgt.resize(Eigen::NoChange, total_size);
    size_t current_idx = 0;
    // std::cerr << total_size << std::endl;
    for (size_t i = 0; i < icps_.size(); ++i) {
      full_src.block(0, current_idx, 3, src[i].cols()) = src[i];
      full_tgt.block(0, current_idx, 3, tgt[i].cols()) = tgt[i];
      current_idx += src[i].cols();
    }

    if (!ICP::getTransformationFromMatchedPoints(full_src, full_tgt,
                                                 &T_src_tgt_)) {
      return false;
    }
  }

  return true;
}
