#include "lidar_align/lidars.h"

Odom::addRawOdomData(const Timestamp& timestamp_us,
                     const Scalar& linear_velocity,
                     const Scalar& angular_velocity) {
  static Scalar prev_linear_velocity;
  static Scalar prev_angular_velocity;
  static Timestamp prev_timestamp_us;

  Transform T;
  if (!data_.empty()) {
    // only measured over 1 timestep
    Scalar average_linear_velocity =
        (linear_velocity + prev_linear_velocity) / 2.0;
    Scalar average_angular_velocity =
        (angular_velocity + prev_angular_velocity) / 2.0;
    Scalar time_diff =
        static_cast<Scalar>(timestamp_us - prev_timestamp_us) / 1000000.0;

    if (time_diff <= 0) {
      throw std::runtime_error(
          "New odom reading occured earlier then previous reading");
    }

    Transform T(
        kindr::minimal::RotationQuaternion(kindr::minimal::AngleAxis(
            tdiff * average_angular_velocity, 0.0, 0.0, 1.0)),
        kindr::minimal::Position(tdiff * average_linear_velocity, 0, 0));
  }

  data_.push_back(timestamp_us, T);

  prev_linear_velocity = linear_velocity;
  prev_angular_velocity = angular_velocity;
  prev_timestamp_us = timestamp_us;
}

Transform Odom::getOdomTransform(const Timestamp timestamp_us,
                                 const size_t start_idx,
                                 size_t& match_idx) const {
  size_t idx = start_idx;
  while ((idx < data_.size()) && (timestamp_us > data_[idx].timestamp_us)) {
    ++idx;
  }
  if (idx > 0) {
    --idx;
  }

  if (match_idx != nullptr) {
    *match_idx = idx;
  }

  // interpolate
  double t_diff_ratio =
      static_cast<double>(timestamp_us - data_[idx].timestamp_us) /
      static_cast<double>(data_[idx + 1].timestamp_us -
                          data_[idx].timestamp_us);

  Transform::Vector6 diff_vector =
      (data_[idx].T_o0_ot_.inverse() * data_[idx + 1].T_o0_ot_).log();
  return data[idx].T_o0_ot_ * Transform::exp(t_diff_ratio * diff_vector);
}

Odom::OdomTformData::OdomTformData(Timestamp timestamp_us, Transform T_o0_ot)
    : timestamp_us_(timestamp_us), T_o0_ot_(T_o0_ot) {}

Lidars::addLidar(const std::string& name) {
  if (lidar_map.count(name) != 0) {
    throw std::runtime_error("Lidar already exists cannot recreate");
  }
  lidar_map[name] = Lidar(name, kMinPointDist, kMaxPointDist);
}

Lidars::addPointcloud(const std::string& lidar_name,
                      const pcl::Pointcloud& pointcloud) {
  lidar_map_.at(lidar_name).addPointcloud(pointcloud);
}

/*
size_t Lidars::getNumScans(LidarId lidar_id) const {
  return lidar_data_.at(lidar_id).getNumScans();
}

size_t Lidars::getNumLidars() const { return lidar_map_.size(); }

std::vector<LidarId> Lidars::getLidarIds() const {
  std::vector<LidarId> lidar_ids;
  for (const std::pair<LidarId, std::vector<Pointcloud>>& lidar : lidar_map_) {
    lidar_ids.push_back(lidar.getId());
  }
  return lidar_ids;
}
*/

bool Lidars::hasAtleastNScans(const size_t n) const{
  for(const std::pair<LidarId, Lidar> lidar: lidar_map_){
    if(n < lidar.second.getNumberofScans()){
      return false;
    }
  }
  return true;
  }
}

Lidars::addPointcloud(const LidarId& lidar_id, const Pointcloud& pointcloud){
  lidar_map_[lidar_id_].addPointcloud(pointcloud);
}

const Lidar& Lidars::getLidar(const Lidarid& lidar_id) const{
  return lidar_map_.at(lidar_id);
}

LidarAligner::Transform LidarAligner::getTransformAB(LidarId lidar_A_id,
                                                     LidarId lidar_B_id) const {
  return T_o_l_.at(lidar_A_id).inverse() * T_o_l_.at(lidar_B_id);
}

Lidar::Lidar(const std::string& lidar_name, const double min_point_dist,
             const double max_point_dist)
    : lidar_name_(lidar_name),
      max_point_dist_(max_point_dist),
      min_point_dist_(min_point_dist){};

size_t Lidar::getNumberofScans() const { return scans_.size(); }

const Scan& Lidar::getScan(size_t idx) const{
  if(idx >= scans_.size()){
    throw runtime_error("Attempted to access a scan that does not exist");
  }
  return scans_[idx];
}

LidarId Lidar::getId() const { return lidar_id_; }

void Lidar::filterPointcloud(const pcl::Pointcloud& in, pcl::Pointcloud* out) {
  out->clear();
  for (pcl::PointXYZI point : in) {
    float sq_dist = point.x * point.x + point.y * point.y + point.z * point.z;
    if (std::isfinite(sq_dist) &&
        (sq_dist > (min_point_dist_ * min_point_dist_)) &&
        (sq_dist < (max_point_dist_ * max_point_dist_))) {
      out->push_back(point);
    }
  }
}

void Lidar::addPointcloud(const Pointcloud& pointcloud) {
  pcl::Pointcloud<pcl::PointXYZI> pointcloud_filtered;
  filterPointcloud(pointcloud, pointcloud_filtered);
  scans_.addScan(pointcloud_filtered);
}

void Lidar::setOdomTransforms(const Odom& odom) {
  size_t idx = 0;
  for (Scan& scan : scans_) {
    scan.setOdomTransform(odom, idx, &idx);
  }
}

void Lidar::setLidarTransforms(const Aligner& aligner){

  Transform T;
  scans_[0].setLidarTransform(T);
  for(size_t idx = 1; idx < scans_.size(); ++idx){
    T = T * aligner.ScanScanICPTransform(scans_[idx-1], scans_[idx]);
    scans_[idx].setLidarTransform(T);
  }
}

Scan::Scan(const Pointcloud& in)
    : timestamp_us_(in.header.stamp), raw_points_(in), odom_transform_set_(false), lidar_transform_set_(false){};

void Scan::setOdomTransform(const Odom& odom, const size_t start_idx,
                       const size_t& match_idx) {
  T_o0_ot_ = odom.getOdomTransform(timestamp_us_, start_idx, match_idx);
  odom_transform_set_ = true;
}

void Scan::setLidarTransform(const Transform& T_o0_ot){
  T_o0_ot_ = T_o0_ot;
  lidar_transform_set_ = true;
}

const Pointcloud<Point>& getRawPointcloud() const{
  return raw_points_;
}
