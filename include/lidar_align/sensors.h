#ifndef LIDAR_ALIGN_SENSORS_H_
#define LIDAR_ALIGN_SENSORS_H_

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

#include <kindr/minimal/quat-transformation.h>

typedef int LidarId;
typedef double Scalar;
// this must be at least 64 bit and signed or things will break
typedef long long int Timestamp;

typedef kindr::minimal::QuatTransformationTemplate<Scalar> Transform;
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
  void addRawOdomData(const Timestamp& timestamp_us,
                      const Scalar& linear_velocity,
                      const Scalar& angular_velocity);

  Transform getOdomTransform(const Timestamp timestamp_us,
                             const size_t start_idx = 0,
                             size_t* match_idx = nullptr) const;

 private:
  std::vector<OdomTformData> data_;
};

class Scan {
 public:
  Scan(const Pointcloud& pointcloud);

  void setOdomTransform(const Odom& odom, const size_t start_idx,
                        size_t* match_idx);

  void setLidarTransform(const Transform& T_o0_ot);

  const Transform& getOdomTransform() const;

  const Pointcloud& getRawPointcloud() const;

 private:
  Timestamp timestamp_us_;  // signed to allow simpler comparisons
  Pointcloud raw_points_;
  Pointcloud synced_points_;
  Transform T_l0_lt_;  // absolute lidar transform at this timestamp
  Transform T_o0_ot_;  // absolute odom transform at this timestamp

  bool synced_;
  bool odom_transform_set_;
  bool lidar_transform_set_;
};

class Lidar {
 public:
  Lidar(const LidarId& lidar_id, const Scalar min_point_dist,
        const Scalar max_point_dist);

  size_t getNumberOfScans() const;

  LidarId getId() const;

  const Scan& getScan(size_t idx) const;

  void addPointcloud(const Pointcloud& pointcloud);

  void setOdomOdomTransforms(const Odom& odom);

  void setLidarLidarTransform(const Transform& T, const size_t& scan_idx);

  void setOdomLidarTransform(const Transform& T_o_l);

  //used for debugging frames
  void saveCombinedPointcloud(const std::string& file_path);

  const Transform& getOdomLidarTransform() const;

 private:
  LidarId lidar_id_;
  Transform T_o_l_;  // transform from lidar to odometry

  std::vector<Scan> scans_;

  Scalar max_point_dist_;
  Scalar min_point_dist_;

  void filterPointcloud(const Pointcloud& in, Pointcloud* out);
};

class Lidars {
 public:
  void addPointcloud(const LidarId& lidar_id, const Pointcloud& pointcloud);

  const Lidar& getLidar(const LidarId& lidar_id) const;

  std::vector<Lidar>& getLidarsRef();

  bool hasAtleastNScans(const size_t n) const;

 private:
  // while it is nice to refer to lidars by id, we often just need a vector of
  // them (thus why the lidars are not in the map directly)
  std::map<LidarId, size_t> id_to_idx_map_;
  std::vector<Lidar> lidar_vector_;

  //static constexpr Scalar kMinPointDist = 1.0;
  //static constexpr Scalar kMaxPointDist = 100.0;
};

#endif