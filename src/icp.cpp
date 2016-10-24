
/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#include <hoop_detect/icp.h>

ICP::ICP() : tgt_pc_ptr_(new pcl::PointCloud<pcl::PointXYZ>) {}

bool ICP::getTransformFromCorrelation(const Eigen::Matrix3Xf &src_demean,
                                      const Eigen::Vector3f &src_center,
                                      const Eigen::Matrix3Xf &tgt_demean,
                                      const Eigen::Vector3f &tgt_center) {
  CHECK(src_demean.cols() == tgt_demean.cols());

  // Assemble the correlation matrix H = source * target'
  Eigen::Matrix3f H = src_demean * tgt_demean.transpose();

  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(
      H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3f u = svd.matrixU();
  Eigen::Matrix3f v = svd.matrixV();

  // Compute R = V * U'
  if (u.determinant() * v.determinant() < 0) {
    for (int x = 0; x < 3; ++x) v(x, 2) *= -1;
  }

  // Form transformation
  Eigen::Matrix3f rotation_matrix = v * u.transpose();
  Eigen::Vector3f trans = tgt_center - (rotation_matrix * src_center);

  if (!Rotation::isValidRotationMatrix(rotation_matrix)) {
    return false;
  }

  T_src_tgt_ = Transformation(Rotation(rotation_matrix), trans);
  return true;
}

bool ICP::getTransformationFromMatchedPoints(const Eigen::Matrix3Xf &src,
                                             const Eigen::Matrix3Xf &tgt) {
  CHECK(src.cols() == tgt.cols());

  // find and remove mean
  Eigen::Vector3f src_center = src.rowwise().mean();
  Eigen::Vector3f tgt_center = tgt.rowwise().mean();

  Eigen::Matrix3Xf src_demean = src.colwise() - src_center;
  Eigen::Matrix3Xf tgt_demean = tgt.colwise() - tgt_center;

  // align
  return getTransformFromCorrelation(src_demean, src_center, tgt_demean,
                                     tgt_center);
}

void ICP::setTargetPoints(const pcl::PointCloud<pcl::PointXYZ> &tgt_pc) {
  tgt_pc_ptr_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(
      *(new pcl::PointCloud<pcl::PointXYZ>));
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(tgt_pc, *tgt_pc_ptr_, indices);
  if (tgt_pc_ptr_->size() != 0) {
    tgt_kdtree_.setInputCloud(tgt_pc_ptr_);
  }
}

void ICP::setSrcPoints(const pcl::PointCloud<pcl::PointXYZ> &src_pc) {
  src_pc_ = src_pc;
}

pcl::PointCloud<pcl::PointXYZ> ICP::getTformedSrcPoints(void) {
  pcl::PointCloud<pcl::PointXYZ> tformed_pc;
  pcl::transformPointCloud(src_pc_, tformed_pc,
                           T_src_tgt_.getTransformationMatrix());

  return tformed_pc;
}

void ICP::setSrcFreeRegions(const pcl::PointCloud<pcl::PointXYZ> &src_free_pc) {
  src_free_pc_ = src_free_pc;
}

ICP::TransformationD ICP::getCurrentTform(void) {
  //apparently the only way to convert from a float to a double minkindr tform
  //without renormalization will almost always trigger invalid rotation check
  RotationD::RotationMatrix R_mat =
      T_src_tgt_.getRotation().getRotationMatrix().cast<double>();
  RotationD R_temp = RotationD::constructAndRenormalize(R_mat);
  TransformationD::Position p_temp = T_src_tgt_.getPosition().cast<double>();
  return TransformationD(R_temp, p_temp);
}

void ICP::setCurrentTform(Transformation T_src_tgt) { T_src_tgt_ = T_src_tgt; }

void ICP::setCurrentTform(TransformationD T_src_tgt) {
  Transformation::TransformationMatrix T_mat =
      T_src_tgt.getTransformationMatrix().cast<float>();
  T_src_tgt_ = Transformation(T_mat);
}

void ICP::matchPoints(Eigen::Matrix3Xf *src, Eigen::Matrix3Xf *tgt,
                      std::vector<float> *dist_error) {
  std::vector<int> kdtree_idx(1);
  std::vector<float> kdtree_dist(1);

  pcl::PointCloud<pcl::PointXYZ> tformed_pc;
  Eigen::Matrix4f tform = T_src_tgt_.getTransformationMatrix();
  pcl::transformPointCloud(src_pc_, tformed_pc, tform);

  // build alignment matrices
  for (size_t i = 0; i < src_pc_.size(); ++i) {
    tgt_kdtree_.nearestKSearch(tformed_pc[i], 1, kdtree_idx, kdtree_dist);

    pcl::PointXYZ matching_point = tgt_pc_ptr_->at(kdtree_idx.front());

    dist_error->at(i) = kdtree_dist.front();

    src->col(i) = Eigen::Vector3f(src_pc_[i].x, src_pc_[i].y, src_pc_[i].z);
    tgt->col(i) =
        Eigen::Vector3f(matching_point.x, matching_point.y, matching_point.z);
  }
}

void ICP::removeOutliers(const Eigen::Matrix3Xf &src,
                         const Eigen::Matrix3Xf &tgt,
                         const std::vector<float> &dist_error, float num_keep,
                         Eigen::Matrix3Xf *src_filt, Eigen::Matrix3Xf *tgt_filt,
                         std::vector<float> *dist_error_filt) {
  if (num_keep > dist_error_filt->size()) {
    ROS_ERROR("Keeping more than all the points, check rejection ratio");
    return;
  } else if (num_keep <= 0) {
    ROS_ERROR("Rejecting all points, check rejection ratio");
  }

  std::vector<size_t> sorted_idx;
  for (size_t i = 0; i < dist_error.size(); ++i) {
    sorted_idx.push_back(i);
  }

  std::sort(sorted_idx.begin(), sorted_idx.end(), [&](size_t i1, size_t i2) {
    return dist_error[i1] < dist_error[i2];
  });

  for (size_t i = 0; i < num_keep; ++i) {
    src_filt->col(i) = src.col(sorted_idx[i]);
    tgt_filt->col(i) = tgt.col(sorted_idx[i]);
    dist_error_filt->at(i) = dist_error[sorted_idx[i]];
  }
}

void ICP::findFreeRegionError(std::vector<float> *free_error) {
  std::vector<int> kdtree_idx(1);
  std::vector<float> kdtree_dist(1);

  pcl::PointCloud<pcl::PointXYZ> tformed_pc;
  Eigen::Matrix4f tform = T_src_tgt_.getTransformationMatrix().cast<float>();
  pcl::transformPointCloud(src_free_pc_, tformed_pc, tform);

  // build alignment matrices
  for (size_t i = 0; i < tformed_pc.size(); ++i) {
    tgt_kdtree_.nearestKSearch(tformed_pc[i], 1, kdtree_idx, kdtree_dist);

    free_error->at(i) = -kdtree_dist.front();
  }
}

bool ICP::pointsMeetCriteria(float points_threshold, float free_threshold,
                             const std::vector<float> &dist_error,
                             const std::vector<float> &free_error) {
  // note free_error is a negitive (done as a bit of a hack to make everything
  // about minimizing)
  if ((dist_error.back() < points_threshold) &&
      (free_error.back() < free_threshold)) {
    return true;
  } else {
    return false;
  }
}

bool ICP::stepICP(float points_threshold, float free_threshold,
                  float inlier_ratio) {
  int npts = src_pc_.size();
  int npts_free = src_free_pc_.size();
  int nkeep = std::ceil(inlier_ratio * npts);
  int nkeep_free = std::ceil(inlier_ratio * npts_free);

  Eigen::Matrix3Xf src(3, npts);
  Eigen::Matrix3Xf tgt(3, npts);

  Eigen::Matrix3Xf src_filt(3, nkeep);
  Eigen::Matrix3Xf tgt_filt(3, nkeep);

  std::vector<float> dist_error(npts);
  std::vector<float> free_error(npts_free);

  std::vector<float> dist_error_filt(nkeep);
  std::vector<float> free_error_filt(nkeep_free);

  if ((tgt_pc_ptr_->size() == 0) || (src_pc_.size() == 0)) {
    return false;
  }

  matchPoints(&src, &tgt, &dist_error);

  findFreeRegionError(&free_error);

  removeOutliers(src, tgt, dist_error, nkeep, &src_filt, &tgt_filt,
                 &dist_error_filt);

  // place holders value not actually needed
  Eigen::Matrix3Xf src_filt_free(3, nkeep_free);
  Eigen::Matrix3Xf tgt_filt_free(3, nkeep_free);

  removeOutliers(src, tgt, free_error, nkeep_free, &src_filt_free,
                 &tgt_filt_free, &free_error_filt);

  if (!getTransformationFromMatchedPoints(src_filt, tgt_filt)) {
    return false;
  }

  return pointsMeetCriteria(points_threshold, -free_threshold, dist_error_filt,
                            free_error_filt);
}

bool ICP::runICP(size_t iterations, float points_threshold,
                 float free_threshold, float inlier_ratio) {
  bool successful = false;
  if ((tgt_pc_ptr_->size() == 0) || (src_pc_.size() == 0) ||
      (src_free_pc_.size() == 0)) {
    return successful;
  }
  for (size_t i = 0; i < iterations; ++i) {
    successful = stepICP(points_threshold, free_threshold, inlier_ratio);
  }
  return successful;
}