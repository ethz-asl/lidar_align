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
#ifndef LIDAR_ALIGN_ICP_H_
#define LIDAR_ALIGN_ICP_H_

#include <kindr/minimal/quat-transformation.h>
#include <algorithm>

#include <ros/ros.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>

#include <boost/make_shared.hpp>

class ICP {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Transformation type for defining sensor orientation. (internally use floats
  // for speed, externally use doubles for compatability)
  typedef kindr::minimal::QuatTransformationTemplate<float> Transformation;
  typedef kindr::minimal::RotationQuaternionTemplate<float> Rotation;
  typedef kindr::minimal::QuatTransformationTemplate<double> TransformationD;
  typedef kindr::minimal::RotationQuaternionTemplate<double> RotationD;

  ICP();

  void setTgtPoints(const pcl::PointCloud<pcl::PointXYZI> &tgt_pc);

  void setSrcPoints(const pcl::PointCloud<pcl::PointXYZI> &src_pc);

  pcl::PointCloud<pcl::PointXYZI> getTformedSrcPoints() const;

  TransformationD getCurrentTform() const;

  void setCurrentTform(Transformation T_src_tgt);

  void setCurrentTform(TransformationD T_src_tgt);

  bool runICP(size_t iterations, float inlier_ratio);

  bool getFilteredMatchingPoints(float inlier_ratio,
                                    Eigen::Matrix3Xf *src_filt,
                                    Eigen::Matrix3Xf *tgt_filt) const;

  static  bool getTransformationFromMatchedPoints(const Eigen::Matrix3Xf &src,
                                          const Eigen::Matrix3Xf &tgt, Transformation* T_src_tgt);

  pcl::PointCloud<pcl::PointXYZI> src_pc_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr tgt_pc_ptr_;
  pcl::KdTreeFLANN<pcl::PointXYZI> tgt_kdtree_;


 private:
  Transformation T_src_tgt_;

  static bool getTransformFromCorrelation(const Eigen::Matrix3Xf &src_demean,
                                   const Eigen::Vector3f &src_center,
                                   const Eigen::Matrix3Xf &tgt_demean,
                                   const Eigen::Vector3f &tgt_center, Transformation* T_src_tgt);

  void matchPoints(Eigen::Matrix3Xf *src, Eigen::Matrix3Xf *tgt,
                   std::vector<float> *dist_error) const;

  static void removeOutliers(const Eigen::Matrix3Xf &src, const Eigen::Matrix3Xf &tgt,
                      const std::vector<float> &dist_error, float num_keep,
                      Eigen::Matrix3Xf *src_filt, Eigen::Matrix3Xf *tgt_filt,
                      std::vector<float> *dist_error_filt);

  bool stepICP(float inlier_ratio);
};

#endif