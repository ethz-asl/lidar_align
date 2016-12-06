#include "lidar_align/lidar_aligner.h"

LidarAligner::LidarAligner(const ros::NodeHandle& nh,
                           const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  nh_private_.param("min_distance_filter", min_distance_filter_,
                    kDefaultMinDistanceFilter);
}

LidarAligner::Scalar LidarAligner::getErrorBetweenTimesteps(
    const Transform T_At_Atp1, LidarId lidar_A_id, size_t t_idx,
    Scalar inlier_ratio) const {
  Scalar total_err = 0;

  pcl::KdTreeFLANN<pcl::PointXYZI> scan_At_kdtree;
  Pointcloud::Ptr scan_At_ptr =
      boost::make_shared<Pointcloud>(*(new Pointcloud));

  pcl::transformPointCloud(
      lidar_data_.at(lidar_A_id)[t_idx], *scan_At_ptr,
      T_At_Atp1.inverse().cast<float>().getTransformationMatrix());

  scan_At_kdtree.setInputCloud(scan_At_ptr);

  std::vector<int> kdtree_idx(1);
  std::vector<float> kdtree_dist(1);

  std::vector<float> raw_error;
  for (pcl::PointXYZI point : lidar_data_.at(lidar_A_id)[t_idx + 1]) {
    scan_At_kdtree.nearestKSearch(point, 1, kdtree_idx, kdtree_dist);
    raw_error.push_back(kdtree_dist[0]);
  }

  std::sort(raw_error.begin(), raw_error.end());
  total_err += std::accumulate(
      raw_error.begin(),
      raw_error.begin() + std::ceil(inlier_ratio * raw_error.size()), 0.0f);

  // pcl::io::savePLYFile("/home/z/datasets/ibeo/A.ply", *scan_At_ptr);
  // pcl::io::savePLYFile("/home/z/datasets/ibeo/B.ply",
  // lidar_data_.at(lidar_A_id)[t_idx+1]);

  return total_err;
}

LidarAligner::Transform LidarAligner::getICPTransformBetweenTimesteps(
    const Transform& T_At_Atp1_inital, const LidarId lidar_A_id,
    const size_t t_idx, const Scalar inlier_ratio,
    const size_t iterations) const {
  ICP icp;
  Transform T_At_Atp1 = T_At_Atp1_inital;

  if (lidar_data_.count(lidar_A_id) == 0) {
    ROS_ERROR_STREAM("Lidar id " << lidar_A_id << " not found");
    return T_At_Atp1;
  }

  if (lidar_data_.at(lidar_A_id).size() <= t_idx + 1) {
    ROS_ERROR_STREAM("Cannot access scan " << (t_idx + 1)
                                           << " of lidar with id " << lidar_A_id
                                           << ", not enough scans");
    return T_At_Atp1;
  }

  icp.setTgtPoints(lidar_data_.at(lidar_A_id)[t_idx]);
  icp.setSrcPoints(lidar_data_.at(lidar_A_id)[t_idx + 1]);
  icp.setCurrentTform(T_At_Atp1);

  if (icp.runICP(iterations, inlier_ratio)) {
    T_At_Atp1 = icp.getCurrentTform();
  }

  return T_At_Atp1;
}

LidarAligner::Transform LidarAligner::getICPTransformBetweenLidars(
    const LidarId lidar_A_id, const LidarId lidar_B_id,
    const Scalar inlier_saftey_margin, const size_t iterations) const {
  MultiICP micp;
  Transform T_A_B = getTransformAB(lidar_A_id, lidar_B_id);

  if (lidar_data_.count(lidar_A_id) == 0) {
    ROS_ERROR_STREAM("Lidar id " << lidar_A_id << " not found");
    return T_A_B;
  }
  if (lidar_data_.count(lidar_B_id) == 0) {
    ROS_ERROR_STREAM("Lidar id " << lidar_B_id << " not found");
    return T_A_B;
  }

  const size_t num_scans = std::min(lidar_data_.at(lidar_A_id).size(),
                                    lidar_data_.at(lidar_B_id).size());

  float inlier_ratio =
      getSensorOverlap(lidar_A_id, lidar_B_id) * inlier_saftey_margin;
  if (inlier_ratio < 0) {
    ROS_ERROR_STREAM("Inlier ratio too small, skipping");
    return T_A_B;
  }

  ROS_ERROR_STREAM("Num scans " << num_scans);
  ROS_ERROR_STREAM("inlier ratio " << inlier_ratio);
  for (size_t i = 0; i < num_scans; ++i) {
    micp.addPointCloud(lidar_data_.at(lidar_A_id)[i],
                       lidar_data_.at(lidar_B_id)[i], inlier_ratio);
  }

  ROS_ERROR_STREAM("T in \n " << T_A_B);
  micp.setCurrentTform(T_A_B);

  if (micp.runMultiICP(iterations)) {
    T_A_B = micp.getCurrentTform();
  }

  return T_A_B;
}

void LidarAligner::updateTformMapFromVec(const std::vector<Scalar>& vec,
                                         const bool skip_first) {
  std::vector<LidarId> lidar_ids = getLidarIds();
  std::vector<Scalar>::const_iterator vec_it = vec.begin();
  for (std::vector<LidarId>::iterator id_it = (lidar_ids.begin() + skip_first);
       id_it != lidar_ids.end(); ++id_it) {
    Transform::Vector6 vec6;
    for (size_t i = 0; i < vec6.size(); ++i) {
      vec6(i) = *vec_it;
      ++vec_it;
    }
    T_o_l_[*id_it] = Transform(vec6);
  }
}

void LidarAligner::getVecFromTformMap(std::vector<Scalar>* vec,
                                      const bool skip_first) const {
  std::vector<LidarId> lidar_ids = getLidarIds();
  vec->clear();
  for (std::vector<LidarId>::iterator it = (lidar_ids.begin() + skip_first);
       it != lidar_ids.end(); ++it) {
    Transform::Vector6 vec6 = T_o_l_.at(*it).log();
    for (size_t i = 0; i < vec6.size(); ++i) {
      vec->push_back(vec6[i]);
    }
  }
}

double LidarAligner::getSensorOverlap(const LidarId lidar_A_id,
                                      const LidarId lidar_B_id) const {
  Scalar angle = T_o_l_.at(lidar_A_id)
                     .getRotation()
                     .getDisparityAngle(T_o_l_.at(lidar_B_id).getRotation());
  double lidar_beam_angle = 3;  // TODO should be calculated from input data
  return (lidar_beam_angle - std::abs(angle)) / lidar_beam_angle;
}

void LidarAligner::setTform(const Transform T_o_l, LidarId lidar_id) {
  T_o_l_[lidar_id] = T_o_l;
}

/*Transform LidarAligner::Vec6ToTform(const Scalar*
const vec6){
  return Vec6ToTform(Transform::Vector6(vec6));
}

Transform LidarAligner::Vec6ToTform(const
Transform::Vector6& vec6){
  return Transform::exp(vec6);
}

Transform LidarAligner::Vec3ToTform(const Scalar* const
vec3){
    return Vec3ToTform(Transform::Vector3(vec3));
}

Transform LidarAligner::Vec3ToTform(const
Transform::Vector3& vec3){
  Transform::Vector6 vec6;
  vec6(0) = vec3(0);
  vec6(1) = vec3(1);
  vec6(5) = vec3(2);
  return Transform::exp(vec6);
}*/

LidarAligner::Scalar LidarAligner::getErrorBetweenTwoLidars(
    LidarId lidar_A_id, LidarId lidar_B_id, Scalar inlier_ratio) const {
  Transform T_A_B = T_o_l_.at(lidar_A_id).inverse() * T_o_l_.at(lidar_B_id);

  Scalar total_err = 0;
  for (size_t i = 0; i < lidar_data_.at(lidar_A_id).size(); ++i) {
    pcl::KdTreeFLANN<pcl::PointXYZI> scan_A_kdtree;
    Pointcloud::Ptr scan_A_ptr =
        boost::make_shared<Pointcloud>(*(new Pointcloud));

    pcl::transformPointCloud(
        lidar_data_.at(lidar_A_id)[i], *scan_A_ptr,
        T_A_B.inverse().cast<float>().getTransformationMatrix());

    // if(i == 1){
    //  pcl::io::savePLYFile("/home/z/datasets/ibeo/A.ply", *scan_A_ptr);
    //  pcl::io::savePLYFile("/home/z/datasets/ibeo/B.ply", lidar_B[i]);
    //}

    scan_A_kdtree.setInputCloud(scan_A_ptr);

    std::vector<LidarId> kdtree_idx(1);
    std::vector<float> kdtree_dist(1);

    std::vector<float> raw_error;
    for (pcl::PointXYZI point : lidar_data_.at(lidar_B_id)[i]) {
      scan_A_kdtree.nearestKSearch(point, 1, kdtree_idx, kdtree_dist);
      raw_error.push_back(kdtree_dist[0]);
    }

    std::sort(raw_error.begin(), raw_error.end());
    size_t num_points = std::ceil(inlier_ratio * raw_error.size());
    total_err += std::accumulate(raw_error.begin(),
                                 raw_error.begin() + num_points, 0.0f) /
                 static_cast<Scalar>(num_points);
  }

  // using mean instead of sum prevents bias towards maximizing sensor overlap
  total_err /= static_cast<Scalar>(lidar_data_.at(lidar_A_id).size());

  return total_err;
}

LidarAligner::Scalar LidarAligner::getErrorBetweenOverlappingLidars(
    Scalar inlier_ratio, Scalar min_overlap) const {
  std::vector<LidarId> lidar_ids = getLidarIds();
  Scalar total_err = 0;
  size_t num_compared = 0;
  for (size_t i = 0; i < lidar_ids.size(); ++i) {
    for (size_t j = 0; j < lidar_ids.size(); ++j) {
      if (i != j) {
        Scalar overlap = getSensorOverlap(i, j);
        if (overlap > min_overlap) {
          ++num_compared;
          total_err += getErrorBetweenTwoLidars(lidar_ids[i], lidar_ids[j],
                                                inlier_ratio * overlap);
        }
      }
    }
  }

  total_err /= static_cast<Scalar>(num_compared);
  return total_err;
}

void LidarAligner::addLidarScan(const Pointcloud& pointcloud,
                                const std::string& lidar_topic) {
  // ugly hack that I will probably regret later
  if (lidar_topic.find("lower") == std::string::npos) {
    return;
  }

  std::string topic_start = "lidar_";

  LidarId lidar_id = std::strtol(
      &lidar_topic[lidar_topic.find(topic_start) + topic_start.size()], nullptr,
      10);
  addLidarScan(pointcloud, lidar_id);
}

void LidarAligner::addLidarScan(const Pointcloud& pointcloud,
                                const LidarId lidar_id) {
  Pointcloud pointcloud_filtered;
  filterPointcloud(pointcloud, &pointcloud_filtered);

  lidar_data_[lidar_id].push_back(pointcloud_filtered);

  // find lidar to lidar tform
  if (lidar_data_[lidar_id].size() > 1) {
    Transform T_inital;

    T_lt_ltp1_[lidar_id].push_back(getICPTransformBetweenTimesteps(
        T_inital, lidar_id, lidar_data_[lidar_id].size() - 2,
        kDefaultInlierRatio, kDefaultIterations));

    // if odom to lidar transform hasn't been loaded
    if (!T_o_l_.count(lidar_id)) {
      // add inital transform
      XmlRpc::XmlRpcValue T_o_l_xml;
      Transform T_o_l;

      std::string lidar_tform_name =
          std::string("T_O_L") + std::to_string(lidar_id);

      if (nh_private_.getParam(lidar_tform_name, T_o_l_xml)) {
        kindr::minimal::xmlRpcToKindr(T_o_l_xml, &T_o_l);
      } else {
        ROS_WARN_STREAM("Parameter " << lidar_tform_name
                                     << " not found, using identity tform");
      }
      T_o_l_[lidar_id] = T_o_l;
    }
  }
}

void LidarAligner::addOdomReading(const pcl::uint64_t timestamp,
                                  const double linear_velocity,
                                  const double angular_velocity) {
  if (T_o0_ot_.empty()) {
    T_o0_ot_.push_back(
        std::make_pair(timestamp, kindr::minimal::QuatTransformation()));
    return;
  }

  double tdiff = (T_o0_ot_.back().first - timestamp) / 1000000.0;

  if (tdiff <= 0) {
    ROS_ERROR("New odom message is older then previous one, rejecting");
    return;
  }

  kindr::minimal::QuatTransformation T(
      kindr::minimal::RotationQuaternion(
          kindr::minimal::AngleAxis(tdiff * angular_velocity, 0.0, 0.0, 1.0)),
      kindr::minimal::Position(tdiff * linear_velocity, 0, 0));

  T_o0_ot_.push_back(std::make_pair(timestamp, T_o0_ot_.back().second * T));
}

LidarAligner::Transform LidarAligner::getOdomTransform(
    const pcl::uint64_t timestamp) const {
  size_t idx = 0;
  while ((idx < T_o0_ot_.size()) && (timestamp > T_o0_ot_[idx].first)) {
    ++idx;
  }
  if (idx > 0) {
    --idx;
  }

  // interpolate
  double t_diff_ratio =
      static_cast<double>(timestamp - T_o0_ot_[idx].first) /
      static_cast<double>(T_o0_ot_[idx + 1].first - T_o0_ot_[idx].first);

  Transform::Vector6 diff_vector =
      (T_o0_ot_[idx].second.inverse() * T_o0_ot_[idx + 1].second).log();
  return T_o0_ot_[idx].second * Transform::exp(t_diff_ratio * diff_vector);
}

LidarAligner::Transform LidarAligner::getTransformAB(LidarId lidar_A_id,
                                                     LidarId lidar_B_id) const {
  return T_o_l_.at(lidar_A_id).inverse() * T_o_l_.at(lidar_B_id);
}

void LidarAligner::syncOdom(){
  odom_data_sync.clear();
  }

bool LidarAligner::getTransformAtAtp1(LidarId lidar_A_id, size_t t_idx,
                                      Transform* T_At_Atp1) const {
  if (lidar_data_.count(lidar_A_id) == 0) {
    return false;
  }
  if (lidar_data_.at(lidar_A_id).size() < t_idx) {
    return false;
  }

  *T_At_Atp1 = T_lt_ltp1_.at(lidar_A_id)[t_idx];
  return true;
}