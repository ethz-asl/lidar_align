#include "lidar_align/lidar_aligner.h"

double LidarAligner::getErrorBetweenTimesteps(const kindr::minimal::QuatTransformation T_At_Atp1,
                              int lidar_A_id, size_t t_idx, double inlier_ratio) const{
  double total_err = 0;

    pcl::KdTreeFLANN<pcl::PointXYZI> scan_At_kdtree;
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_At_ptr =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(
            *(new pcl::PointCloud<pcl::PointXYZI>));

    pcl::transformPointCloud(lidar_data_.at(lidar_A_id)[t_idx], *scan_At_ptr,
                             T_At_Atp1.inverse().cast<float>().getTransformationMatrix());


    scan_At_kdtree.setInputCloud(scan_At_ptr);

    std::vector<int> kdtree_idx(1);
    std::vector<float> kdtree_dist(1);

    std::vector<float> raw_error;
    for (pcl::PointXYZI point : lidar_data_.at(lidar_A_id)[t_idx+1]) {
      scan_At_kdtree.nearestKSearch(point, 1, kdtree_idx, kdtree_dist);
      raw_error.push_back(kdtree_dist[0]);
    }

    std::sort(raw_error.begin(), raw_error.end());
    total_err += std::accumulate(
        raw_error.begin(),
        raw_error.begin() + std::ceil(inlier_ratio * raw_error.size()), 0.0f);

    //pcl::io::savePLYFile("/home/z/datasets/ibeo/A.ply", *scan_At_ptr);
    //pcl::io::savePLYFile("/home/z/datasets/ibeo/B.ply", lidar_data_.at(lidar_A_id)[t_idx+1]);

  return total_err;
}

kindr::minimal::QuatTransformation LidarAligner::getICPTransformBetweenTimesteps(
    const kindr::minimal::QuatTransformation T_At_Atp1_in, int lidar_A_id,
    size_t t_idx, double inlier_ratio, size_t iterations) const{

  ICP icp;
  kindr::minimal::QuatTransformation T_At_Atp1_out = T_At_Atp1_in;

  if(lidar_data_.count(lidar_A_id) == 0){
    ROS_ERROR_STREAM("Lidar id " << lidar_A_id  << " not found");
    return T_At_Atp1_out;
  }

  if(lidar_data_.at(lidar_A_id).size() <= t_idx+1){
    ROS_ERROR_STREAM("Cannot access scan " << (t_idx+1) << " of lidar with id " << lidar_A_id << ", not enough scans");
    return T_At_Atp1_out;
  }

  icp.setTargetPoints(lidar_data_.at(lidar_A_id)[t_idx]);
  icp.setSrcPoints(lidar_data_.at(lidar_A_id)[t_idx+1]);
  icp.setCurrentTform(T_At_Atp1_in);

  if(icp.runICP(iterations, inlier_ratio)){
    T_At_Atp1_out = icp.getCurrentTform();
  }

  return T_At_Atp1_out;
}

kindr::minimal::QuatTransformation LidarAligner::Vec6ToTform(const double* const vec6){
  return Vec6ToTform(kindr::minimal::QuatTransformation::Vector6(vec6));
}

kindr::minimal::QuatTransformation LidarAligner::Vec6ToTform(const kindr::minimal::QuatTransformation::Vector6& vec6){
  return kindr::minimal::QuatTransformation::exp(vec6);
}

kindr::minimal::QuatTransformation LidarAligner::Vec3ToTform(const double* const vec3){
    return Vec3ToTform(kindr::minimal::QuatTransformation::Vector3(vec3));
}

kindr::minimal::QuatTransformation LidarAligner::Vec3ToTform(const kindr::minimal::QuatTransformation::Vector3& vec3){
  kindr::minimal::QuatTransformation::Vector6 vec6;
  vec6(0) = vec3(0);
  vec6(1) = vec3(1);
  vec6(5) = vec3(2);
  return kindr::minimal::QuatTransformation::exp(vec6);
}

double LidarAligner::getErrorBetweenLidars(const kindr::minimal::QuatTransformation T_A_B,
                              int lidar_A_id, int lidar_B_id, double inlier_ratio) const{
  double total_err = 0;
  for (size_t i = 0; i < lidar_data_.at(lidar_A_id).size(); ++i) {
    pcl::KdTreeFLANN<pcl::PointXYZI> scan_A_kdtree;
    pcl::PointCloud<pcl::PointXYZI>::Ptr scan_A_ptr =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(
            *(new pcl::PointCloud<pcl::PointXYZI>));

    pcl::transformPointCloud(lidar_data_.at(lidar_A_id)[i], *scan_A_ptr,
                             T_A_B.inverse().cast<float>().getTransformationMatrix());

    // if(i == 1){
    //  pcl::io::savePLYFile("/home/z/datasets/ibeo/A.ply", *scan_A_ptr);
    //  pcl::io::savePLYFile("/home/z/datasets/ibeo/B.ply", lidar_B[i]);
    //}

    scan_A_kdtree.setInputCloud(scan_A_ptr);

    std::vector<int> kdtree_idx(1);
    std::vector<float> kdtree_dist(1);

    std::vector<float> raw_error;
    for (pcl::PointXYZI point : lidar_data_.at(lidar_B_id)[i]) {
      scan_A_kdtree.nearestKSearch(point, 1, kdtree_idx, kdtree_dist);
      raw_error.push_back(kdtree_dist[0]);
    }

    std::sort(raw_error.begin(), raw_error.end());
    total_err += std::accumulate(
        raw_error.begin(),
        raw_error.begin() + std::ceil(inlier_ratio * raw_error.size()), 0.0f);
  }

  return total_err;
}

void LidarAligner::filterPointcloud(const pcl::PointCloud<pcl::PointXYZI>& in,
                                    pcl::PointCloud<pcl::PointXYZI>* out) {
  out->clear();
  for (pcl::PointXYZI point : in) {
    float sq_dist = point.x * point.x + point.y * point.y + point.z * point.z;
    if (std::isfinite(sq_dist) && sq_dist > (min_distance_filter_ * min_distance_filter_)) {
      out->push_back(point);
    }
  }
}

bool LidarAligner::operator()(
    const double* const tform_vec, double* residual) const {

  kindr::minimal::QuatTransformation tform = Vec6ToTform(tform_vec);
  residual[0] = getErrorBetweenLidars(tform, 1, 3, 0.2);

  return true;
}

LidarAligner::LidarAligner(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  nh_private_.param("min_distance_filter", min_distance_filter_, kDefaultMinDistanceFilter);
}

void LidarAligner::addLidarScan(
    const pcl::PointCloud<pcl::PointXYZI>& pointcloud,
    const std::string& lidar_topic) {

  //ugly hack that I will probably regret later
  if(lidar_topic.find("upper") == std::string::npos){
    return;
  }

  std::string topic_start = "lidar_";

  int lidar_id = std::strtol(&lidar_topic[lidar_topic.find(topic_start) + topic_start.size()], nullptr, 10);
  addLidarScan(pointcloud, lidar_id);
}

void LidarAligner::addLidarScan(
    const pcl::PointCloud<pcl::PointXYZI>& pointcloud, const int lidar_id) {
  pcl::PointCloud<pcl::PointXYZI> pointcloud_filtered;
  filterPointcloud(pointcloud, &pointcloud_filtered);

  lidar_data_[lidar_id].push_back(pointcloud_filtered);

  //if transform hasn't been loaded
  if (!tforms_.count(lidar_id)){
    //add inital transform
    XmlRpc::XmlRpcValue T_odom_lidar_xml;
    kindr::minimal::QuatTransformation T_odom_lidar;

    std::string lidar_tform_name = std::string("T_O_L") + std::to_string(lidar_id);

    if (nh_private_.getParam(lidar_tform_name, T_odom_lidar_xml)) {
      kindr::minimal::xmlRpcToKindr(T_odom_lidar_xml, &T_odom_lidar);
    } else {
      ROS_WARN_STREAM("Parameter " << lidar_tform_name
                                     << " not found, using identity tform");
    }
    tforms_[lidar_id] = T_odom_lidar;
  }
}

bool LidarAligner::hasAtleastNScans(size_t n) const{

  if(lidar_data_.size() == 0){
    return false;
  }
  for (const std::pair<const int, std::vector<pcl::PointCloud<pcl::PointXYZI>>>&
           lidar_scans :
       lidar_data_) {
    if (n > lidar_scans.second.size()) {
      return false;
    }
  }

  return true;
}

size_t LidarAligner::getNumFrames(int lidar_id) const{
  return lidar_data_.at(lidar_id).size();
}

std::vector<int> LidarAligner::getLidarIds() const{
  std::vector<int> lidar_ids;
  for (const std::pair<const int, std::vector<pcl::PointCloud<pcl::PointXYZI>>>&
           lidar_scans :
       lidar_data_) {
    lidar_ids.push_back(lidar_scans.first);
  }
  return lidar_ids;
}

kindr::minimal::QuatTransformation LidarAligner::getTransformAB(int lidar_A_id, int lidar_B_id){
  return tforms_[lidar_A_id].inverse() * tforms_[lidar_B_id];
}