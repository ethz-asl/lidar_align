#ifndef LIDAR_ALIGN_SENSORS_H_
#define LIDAR_ALIGN_SENSORS_H_

#include <random>


#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <kindr/minimal/quat-transformation.h>

typedef std::string LidarId;
typedef float Scalar;
// this must be at least 64 bit and signed or things will break
typedef long long int Timestamp;

typedef kindr::minimal::QuatTransformationTemplate<Scalar> Transform;
typedef kindr::minimal::AngleAxisTemplate<Scalar> AngleAxis;
typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Pointcloud;

class OdomTformData {
 public:
  OdomTformData(Timestamp timestamp_us, Transform T_o0_ot);

  const Transform& getTransform() const;
  const Timestamp& getTimestamp() const;

 private:
  Transform T_o0_ot_;
  Timestamp timestamp_us_;
};

class Odom {
 public:
  void addTransformData(const Timestamp& timestamp_us,
                        const Transform& transform);

  Transform getOdomTransform(const Timestamp timestamp_us,
                             const size_t start_idx = 0,
                             size_t* match_idx = nullptr) const;

 private:
  std::vector<OdomTformData> data_;
};

class Scan {
 public:
  struct Config {
    // set default values
    Config() {
      min_point_distance = 0;
      max_point_distance = 100;
      keep_points_ratio = 0.01;
      min_return_intensity = 5.0;

      clockwise_lidar = false;
      motion_compensation = false;
      lidar_rpm = 600.0;
    }

    Scalar min_point_distance;
    Scalar max_point_distance;
    Scalar keep_points_ratio;
    Scalar min_return_intensity;

    bool clockwise_lidar;
    bool motion_compensation;
    Scalar lidar_rpm;
  };

  Scan(const Pointcloud& pointcloud, const Config& config = Config());

  static Config getConfig(ros::NodeHandle* nh);

  void setOdomTransform(const Odom& odom, const double time_offset,
                        const size_t start_idx, size_t* match_idx);

  const Transform& getOdomTransform() const;

  const Pointcloud& getRawPointcloud() const;

  void getTimeAlignedPointcloud(const Transform& T_o_l,
                                Pointcloud* pointcloud) const;

 private:
  Timestamp timestamp_us_;  // signed to allow simpler comparisons
  Pointcloud raw_points_;
  std::vector<Transform>
      T_o0_ot_;  // absolute odom transform to each point in pointcloud

  bool odom_transform_set_;
};

class Lidar {
 public:
  Lidar(const LidarId& lidar_id = "Lidar");

  const size_t getNumberOfScans() const;

  // note points are appended so any points in *pointcloud are preserved
  void getCombinedPointcloud(Pointcloud* pointcloud) const;

  const LidarId& getId() const;

  void addPointcloud(const Pointcloud& pointcloud,
                     const Scan::Config& config = Scan::Config());

  void setOdomOdomTransforms(const Odom& odom, const double time_offset = 0.0);

  void setOdomLidarTransform(const Transform& T_o_l);

  // used for debugging frames
  void saveCombinedPointcloud(const std::string& file_path) const;

  const Transform& getOdomLidarTransform() const;

 private:
  LidarId lidar_id_;
  Transform T_o_l_;  // transform from lidar to odometry

  std::vector<Scan> scans_;
};

#endif