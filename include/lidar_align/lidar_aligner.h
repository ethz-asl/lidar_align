#ifndef LIDAR_ALIGNER_H
#define LIDAR_ALIGNER_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/TransformStamped.h>
#include <kindr/minimal/position.h>
#include <kindr/minimal/quat-transformation.h>

#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>
#include <minkindr_conversions/kindr_xml.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <limits>

#include "lidar_align/icp.h"

class LidarAligner {
 public:
  LidarAligner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  bool operator()(const double* const tform_vec, double* residual) const;

  void addLidarScan(const pcl::PointCloud<pcl::PointXYZI>& pointcloud,
                    const std::string& lidar_topic);

  void addLidarScan(const pcl::PointCloud<pcl::PointXYZI>& pointcloud,
                    const int lidar_id);

  double getErrorBetweenLidars(
      const kindr::minimal::QuatTransformation T_A_B, int lidar_A_id,
      int lidar_B_id, double inlier_ratio) const;

  double getErrorBetweenTimesteps(
      const kindr::minimal::QuatTransformation T_At_Atp1, int lidar_A_id,
      size_t t_idx, double inlier_ratio) const;

kindr::minimal::QuatTransformation getICPTransformBetweenTimesteps(
    const kindr::minimal::QuatTransformation T_At_Atp1_in, int lidar_A_id,
    size_t t_idx, double inlier_ratio, size_t iterations) const;

  bool hasAtleastNScans(size_t n) const;

  std::vector<int> getLidarIds() const;

  size_t getNumFrames(int lidar_id) const;

  kindr::minimal::QuatTransformation getTransformAB(int lidar_A_id,
                                                    int lidar_B_id);

  static kindr::minimal::QuatTransformation Vec6ToTform(
      const double* const vec6);

  static kindr::minimal::QuatTransformation Vec6ToTform(
      const kindr::minimal::QuatTransformation::Vector6& vec6);

  static kindr::minimal::QuatTransformation Vec3ToTform(
      const double* const vec3);

  static kindr::minimal::QuatTransformation Vec3ToTform(
      const kindr::minimal::QuatTransformation::Vector3& vec3);

 private:
  // minimum distance a point can be from the scanner and be considered valid
  // (set negative to disable)
  static constexpr double kDefaultMinDistanceFilter = 2;
  double min_distance_filter_;

  std::map<int, std::vector<pcl::PointCloud<pcl::PointXYZI>>> lidar_data_;
  std::map<int, kindr::minimal::QuatTransformation> tforms_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  void filterPointcloud(const pcl::PointCloud<pcl::PointXYZI>& in,
                        pcl::PointCloud<pcl::PointXYZI>* out);
  void get_init_tforms();
};

#endif //LIDAR_ALIGNER_H