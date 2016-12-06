#ifndef LIDAR_ALIGN_LIDARS_H_
#define LIDAR_ALIGN_LIDARS_H_

typedef int LidarId;
typedef Scalar Scalar;
// this must be at least 64 bit and signed or things will break
typedef long long int Timestamp;

typedef kindr::minimal::QuatTransformationTemplate<Scalar> Transform;
typedef pcl::PointXYZI Point;
typedef pcl::PointCloud<Point> Pointcloud;

class Odom {
 public:
  void addRawOdomData(const Timestamp& timestamp_us,
                      const Scalar& linear_velocity,
                      const Scalar& angular_velocity);

  Transform getOdomTransform(const Timestamp timestamp_us,
                             const size_t start_idx = 0,
                             size_t& match_idx = nullptr) const;

 private:
  struct OdomTformData {
    OdomTformData(timestamp_us, T_o0_ot)
        : timestamp_us_(timestamp_us), T_o0_ot_(T_o0_ot){};
    Transform T_o0_ot_;
    Timestamp timestamp_us_;
  }

  std::vector<OdomTformData>
      data_;

}

class Lidars {
 public:

  void addPointcloud(const LidarId& lidar_id, const Pointcloud& pointcloud);

  const Lidar& getLidar(const Lidarid& lidar_id) const;

  bool hasAtleastNScans(const size_t n) const;

 private:
  std::map<LidarId, Lidar> lidar_map_;

  static constexpr Scalar kMinPointDist = 1.0;
  static constexpr Scalar kMaxPointDist = 100.0;
}

class Lidar {
 public:
  Lidar(const LidarId& id, const Scalar max_point_dist,
        const Scalar min_point_dist);

  size_t getNumberOfScans() const;

  const Scan& getScan(size_t idx) const;

  void addPointcloud(const Pointcloud& pointcloud);

  void setOdomTransforms(const Odom& odom);

 private:
  LidarId lidar_id_;
  Transform T_o_l_;  // transform from lidar to odometry

  std::vector<Scan> scans_;

  Scalar max_point_dist_;
  Scalar min_point_dist_;

  void filterPointcloud(const Pointcloud& in, Pointcloud* out);

}

class Scan {
 public:
  Scan(const Pointcloud& pointcloud);

  void setOdomTransform(const Odom& odom, const size_t start_idx,
                        const size_t& match_idx);

  const Pointcloud<Point>& getRawPointcloud() const;

 private:
  Timestamp timestamp_us_;  // signed to allow simpler comparisons
  Pointcloud<Point> raw_points_;
  Pointcloud<Point> synced_points_;
  Transform T_l0_lt_;  // absolute lidar transform at this timestamp
  Transform T_o0_ot_;  // absolute odom transform at this timestamp

  bool synced_;
  bool odom_transform_set_;
  bool lidar_transform_set_;
}

#endif