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
#include "lidar_align/multi_icp.h"

constexpr double kDefaultInlierRatio = 0.7;
constexpr size_t kDefaultIterations = 20;

class LidarAligner {
 public:
  typedef int LidarId;
  typedef double Scalar;

  typedef kindr::minimal::QuatTransformationTemplate<Scalar> Transform;
  typedef pcl::PointCloud<pcl::PointXYZI> Pointcloud;

  LidarAligner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

  void addLidarScan(const Pointcloud& pointcloud,
                    const std::string& lidar_topic);

  void addLidarScan(const Pointcloud& pointcloud, const LidarId lidar_id);

  Scalar getErrorBetweenTimesteps(const Transform T_At_Atp1, LidarId lidar_A_id,
                                  size_t t_idx, Scalar inlier_ratio) const;

  Transform getICPTransformBetweenTimesteps(const Transform& T_At_Atp1_inital,
                                            const LidarId lidar_A_id,
                                            const size_t t_idx,
                                            const Scalar inlier_ratio,
                                            const size_t iterations) const;

  Transform getICPTransformBetweenLidars(const LidarId lidar_A_id,
                                         const LidarId lidar_B_id,
                                         const Scalar inlier_saftey_margin,
                                         const size_t iterations) const;

  Scalar getErrorBetweenOverlappingLidars(Scalar inlier_ratio,
                                          Scalar min_overlap) const;

  bool hasAtleastNScans(size_t n) const;

  std::vector<LidarId> getLidarIds() const;

  size_t getNumScans(LidarId lidar_id) const;

  size_t getNumLidars() const;

  void updateTformMapFromVec(const std::vector<Scalar>& vec,
                             const bool skip_first = false);

  void getVecFromTformMap(std::vector<Scalar>* vec,
                          const bool skip_first = false) const;

  void setTform(const Transform T_l_o, int lidar_id);

  Transform getTransformAB(LidarId lidar_A_id, LidarId lidar_B_id) const;

  bool getTransformAtAtp1(LidarId lidar_A_id, size_t t_idx,
                          Transform* T_At_Atp1) const;

  double getSensorOverlap(LidarId lidar_A_id, LidarId lidar_B_id) const;

  /*static kindr::minimal::QuatTransformation Vec6ToTform(
      const Scalar* const vec6);

  static kindr::minimal::QuatTransformation Vec6ToTform(
      const kindr::minimal::QuatTransformation::Vector6& vec6);

  static kindr::minimal::QuatTransformation Vec3ToTform(
      const Scalar* const vec3);

  static kindr::minimal::QuatTransformation Vec3ToTform(
      const kindr::minimal::QuatTransformation::Vector3& vec3);*/

 private:
  // minimum distance a point can be from the scanner and be considered valid
  // (set negative to disable)
  static constexpr Scalar kDefaultMinDistanceFilter = 2;
  Scalar min_distance_filter_;

  std::map<LidarId, std::vector<Pointcloud>> lidar_data_;
  std::map<LidarId, Transform> T_o_l_;  // odom to lidar transform
  std::map<LidarId, std::vector<Transform>>
      T_lt_ltp1_;  // lidar to same lidar at next timestep transform
  // std::map<LidarId, std::vector<Pointcloud>> lidar_data_sync_;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  Scalar getErrorBetweenTwoLidars(LidarId lidar_A_id, LidarId lidar_B_id,
                                  Scalar inlier_ratio) const;

  void filterPointcloud(const Pointcloud& in, Pointcloud* out);

  void get_init_tforms();
};

#endif  // LIDAR_ALIGNER_H