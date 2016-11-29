#ifndef LIDAR_ALIGN_MULTI_ICP_H_
#define LIDAR_ALIGN_MULTI_ICP_H_

#include "lidar_align/icp.h"

class MultiICP {
 public:

  MultiICP();

  void addPointCloud(const pcl::PointCloud<pcl::PointXYZI> &src_pc, const pcl::PointCloud<pcl::PointXYZI> &tgt_pc, float inlier_ratio);

  void clearPointClouds();

  ICP::TransformationD getCurrentTform() const;

  void setCurrentTform(ICP::Transformation T_src_tgt);

  void setCurrentTform(ICP::TransformationD T_src_tgt);

  bool runMultiICP(size_t iterations);

 private:
  ICP::Transformation T_src_tgt_;
  std::vector<float> inlier_ratios_;
  std::vector<ICP> icps_;
};

#endif