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
  static Transform T;

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

    T = T * Transform(
                Transform::Rotation(AngleAxis(
                    time_diff * average_angular_velocity, 0.0, 0.0, 1.0)),
                Transform::Position(time_diff * average_linear_velocity, 0, 0));
  }

  data_.emplace_back(timestamp_us, T);
  prev_linear_velocity = linear_velocity;
  prev_angular_velocity = angular_velocity;
  prev_timestamp_us = timestamp_us;
}

void Odom::addTransformData(const Timestamp& timestamp_us, const Transform& T) {
  data_.emplace_back(timestamp_us, T);
}

Transform Odom::getOdomTransform(const Timestamp timestamp_us,
                                 const size_t start_idx,
                                 size_t* match_idx) const {
  size_t idx = start_idx;

  while ((idx < (data_.size() - 1)) &&
         (timestamp_us > data_[idx].getTimestamp())) {
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

bool Odom::getFinalAngularVeloctiy(Scalar* angular_velocity) const {
  if (data_.size() < 2) {
    return false;
  }

  const OdomTformData& last_odom = data_.back();
  const OdomTformData& second_to_last_odom = data_.rbegin()[1];

  Scalar angle_diff = last_odom.getTransform().getRotation().getDisparityAngle(
      second_to_last_odom.getTransform().getRotation());
  Scalar time_diff = static_cast<Scalar>(last_odom.getTimestamp() -
                                         second_to_last_odom.getTimestamp()) /
                     1000000.0;
  ;
  *angular_velocity = angle_diff / time_diff;
  return true;
}

Scan::Scan(const Pointcloud& in, const Config& config)
    : timestamp_us_(in.header.stamp), odom_transform_set_(false) {
  std::default_random_engine generator(in.header.stamp);
  std::uniform_real_distribution<float> distribution(0, 1);

  for (const Point& point : in) {
    // break;
    if (distribution(generator) > config.keep_points_ratio) {
      continue;
    }
    float sq_dist = point.x * point.x + point.y * point.y + point.z * point.z;
    if (std::isfinite(sq_dist) &&
        (sq_dist > (config.min_point_distance * config.min_point_distance)) &&
        (sq_dist < (config.max_point_distance * config.max_point_distance))) {
      raw_points_.push_back(point);
    }
  }
  raw_points_.header = in.header;
}

void Scan::setOdomTransform(const Odom& odom, const size_t start_idx,
                            size_t* match_idx) {
  T_o0_ot_.clear();

  size_t i = 0;
  for (Point point : raw_points_) {
    // NOTE: This static cast is really really important. Without it the
    // timestamp_us will be cast to a float, as it is a very large number it
    // will have quite low precision and when it is cast back to a long int
    // will
    // be a very different value (about 2 to 3 million lower in some quick
    // tests). This difference will then break everything.
    Timestamp point_ts_us =
        timestamp_us_ + static_cast<Timestamp>(std::round(point.intensity));
    T_o0_ot_.push_back(
        odom.getOdomTransform(point_ts_us, start_idx, match_idx));
  }
  odom_transform_set_ = true;
}

const Transform& Scan::getOdomTransform() const {
  if (!odom_transform_set_) {
    throw std::runtime_error(
        "Attempted to get odom transform before it was set");
  }
  return T_o0_ot_.front();
}

void Scan::getTimeAlignedPointcloud(const Transform& T_o_l,
                                    Pointcloud* pointcloud) const {
  for (size_t i = 0; i < raw_points_.size(); ++i) {
    Transform T_o_lt = T_o0_ot_[i] * T_o_l;

    Eigen::Affine3f pcl_transform;
    pcl_transform.matrix() = T_o_lt.cast<float>().getTransformationMatrix();

    pointcloud->push_back(pcl::transformPoint(raw_points_[i], pcl_transform));
  }
}

Lidar::Lidar(const LidarId& lidar_id) : lidar_id_(lidar_id){};

const size_t Lidar::getNumberOfScans() const { return scans_.size(); }

const LidarId& Lidar::getId() const { return lidar_id_; }

void Lidar::addPointcloud(const Pointcloud& pointcloud,
                          const Scan::Config& config) {
  scans_.emplace_back(pointcloud, config);
}

void Lidar::getCombinedPointcloud(Pointcloud* pointcloud) const {
  for (const Scan& scan : scans_) {
    scan.getTimeAlignedPointcloud(getOdomLidarTransform(), pointcloud);
  }
}

void Lidar::saveCombinedPointcloud(const std::string& file_path) const {
  Pointcloud combined;

  getCombinedPointcloud(&combined);

  pcl::PLYWriter writer;
  writer.write(file_path, combined, true);
}

void Lidar::setOdomOdomTransforms(const Odom& odom) {
  size_t idx = 0;
  for (Scan& scan : scans_) {
    scan.setOdomTransform(odom, idx, &idx);
  }
}

void Lidar::setOdomLidarTransform(const Transform& T_o_l) { T_o_l_ = T_o_l; }

const Transform& Lidar::getOdomLidarTransform() const { return T_o_l_; }

const size_t LidarArray::getNumberOfLidars() const {
  return lidar_vector_.size();
}

const Lidar& LidarArray::getLidar(const LidarId& lidar_id) const {
  return lidar_vector_[id_to_idx_map_.at(lidar_id)];
}

bool LidarArray::hasAtleastNScans(const size_t n) const {
  if (lidar_vector_.empty()) {
    return false;
  }
  for (const Lidar& lidar : lidar_vector_) {
    if (n > lidar.getNumberOfScans()) {
      return false;
    }
  }
  return true;
}

void LidarArray::addPointcloud(const LidarId& lidar_id,
                               const Pointcloud& pointcloud,
                               const Scan::Config& config) {
  if (id_to_idx_map_.count(lidar_id) == 0) {
    lidar_vector_.emplace_back(lidar_id);
    id_to_idx_map_[lidar_id] = lidar_vector_.size() - 1;
  }
  lidar_vector_[id_to_idx_map_.at(lidar_id)].addPointcloud(pointcloud, config);
}

void LidarArray::getCombinedPointcloud(Pointcloud* pointcloud) const {
  for (const Lidar& lidar : lidar_vector_) {
    lidar.getCombinedPointcloud(pointcloud);
  }
}

std::vector<Lidar>& LidarArray::getLidarVector() { return lidar_vector_; }

const std::vector<Lidar>& LidarArray::getLidarVector() const {
  return lidar_vector_;
}

void LidarArray::setOdomOdomTransforms(const Odom& odom) {
  for (Lidar& lidar : lidar_vector_) {
    lidar.setOdomOdomTransforms(odom);
  }
}
