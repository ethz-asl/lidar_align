#include "lidar_align/sensors.h"

OdomTformData::OdomTformData(Timestamp timestamp_us, Transform T_o0_ot)
    : timestamp_us_(timestamp_us), T_o0_ot_(T_o0_ot) {}

const Transform& OdomTformData::getTransform() const { return T_o0_ot_; }

const Timestamp& OdomTformData::getTimestamp() const { return timestamp_us_; }

void Odom::addRawOdomData(const Timestamp& timestamp_us,
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

    T = Transform(
        kindr::minimal::RotationQuaternion(kindr::minimal::AngleAxis(
            time_diff * average_angular_velocity, 0.0, 0.0, 1.0)),
        kindr::minimal::Position(time_diff * average_linear_velocity, 0, 0));
  }

  data_.emplace_back(timestamp_us, T);

  prev_linear_velocity = linear_velocity;
  prev_angular_velocity = angular_velocity;
  prev_timestamp_us = timestamp_us;
}

Transform Odom::getOdomTransform(const Timestamp timestamp_us,
                                 const size_t start_idx,
                                 size_t* match_idx) const {
  size_t idx = start_idx;

  while ((idx < data_.size()) && (timestamp_us > data_[idx].getTimestamp())) {
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
      static_cast<double>(timestamp_us - data_[idx].getTimestamp()) /
      static_cast<double>(data_[idx + 1].getTimestamp() -
                          data_[idx].getTimestamp());

  Transform::Vector6 diff_vector =
      (data_[idx].getTransform().inverse() * data_[idx + 1].getTransform())
          .log();
  return data_[idx].getTransform() * Transform::exp(t_diff_ratio * diff_vector);
}

Scan::Scan(const Pointcloud& in)
    : timestamp_us_(in.header.stamp),
      raw_points_(in),
      odom_transform_set_(false),
      lidar_transform_set_(false){};

void Scan::setOdomTransform(const Odom& odom, const size_t start_idx,
                            size_t* match_idx) {
  T_o0_ot_ = odom.getOdomTransform(timestamp_us_, start_idx, match_idx);
  odom_transform_set_ = true;
}

void Scan::setLidarTransform(const Transform& T_o0_ot) {
  T_o0_ot_ = T_o0_ot;
  lidar_transform_set_ = true;
}

const Pointcloud& Scan::getRawPointcloud() const { return raw_points_; }

const Transform& Scan::getOdomTransform() const { return T_o0_ot_; }

Lidar::Lidar(const LidarId& lidar_id, const double min_point_dist,
             const double max_point_dist)
    : lidar_id_(lidar_id),
      max_point_dist_(max_point_dist),
      min_point_dist_(min_point_dist){};

size_t Lidar::getNumberOfScans() const { return scans_.size(); }

const Scan& Lidar::getScan(size_t idx) const {
  if (idx >= scans_.size()) {
    throw std::runtime_error("Scan index is out of bounds");
  }
  return scans_[idx];
}

LidarId Lidar::getId() const { return lidar_id_; }

void Lidar::filterPointcloud(const Pointcloud& in, Pointcloud* out) {
  out->clear();
  for (Point point : in) {
    float sq_dist = point.x * point.x + point.y * point.y + point.z * point.z;
    if (std::isfinite(sq_dist) &&
        (sq_dist > (min_point_dist_ * min_point_dist_)) &&
        (sq_dist < (max_point_dist_ * max_point_dist_))) {
      out->push_back(point);
    }
  }
}

void Lidar::addPointcloud(const Pointcloud& pointcloud) {
  Pointcloud pointcloud_filtered;
  filterPointcloud(pointcloud, &pointcloud_filtered);
  scans_.push_back(pointcloud_filtered);
}

void Lidar::setOdomOdomTransforms(const Odom& odom) {
  size_t idx = 0;
  for (Scan& scan : scans_) {
    scan.setOdomTransform(odom, idx, &idx);
  }
}

void Lidar::setLidarLidarTransform(const Transform& T, const size_t& scan_idx) {
  scans_[scan_idx].setLidarTransform(T);
}

void Lidar::setOdomLidarTransform(const Transform& T_o_l){
  T_o_l_ = T_o_l;
}

const Transform& Lidar::getOdomLidarTransform() const { return T_o_l_; }

bool Lidars::hasAtleastNScans(const size_t n) const {
  if (lidar_vector_.empty()) {
    return false;
  }
  for (const Lidar lidar : lidar_vector_) {
    if (n > lidar.getNumberOfScans()) {
      return false;
    }
  }
  return true;
}

void Lidars::addPointcloud(const LidarId& lidar_id,
                           const Pointcloud& pointcloud) {
  if (id_to_idx_map_.count(lidar_id) == 0) {
    lidar_vector_.emplace_back(lidar_id, 1.0, 100.0);
    id_to_idx_map_[lidar_id] = lidar_vector_.size()-1;
  }
  lidar_vector_[id_to_idx_map_.at(lidar_id)].addPointcloud(pointcloud);
}

const Lidar& Lidars::getLidar(const LidarId& lidar_id) const {
  return lidar_vector_[id_to_idx_map_.at(lidar_id)];
}

std::vector<Lidar>& Lidars::getLidarsRef() { return lidar_vector_; }
