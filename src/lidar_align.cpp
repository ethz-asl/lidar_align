#include "lidar_align/lidar_align.h"

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

  return total_err;
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
  nh_private_.param("use_n_frames", use_n_frames_, kDefaultUseNFrames);
  nh_private_.param("inlier_ratio", inlier_ratio_, kDefaultInlierRatio);
  nh_private_.param("min_distance_filter", min_distance_filter_, kDefaultMinDistanceFilter);
}

void LidarAligner::addLidarScan(
    const pcl::PointCloud<pcl::PointXYZI>& pointcloud,
    const std::string& lidar_topic) {

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

    ROS_ERROR_STREAM("loading");
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

bool LidarAligner::haveEnoughScans() {
  for (std::pair<const int, std::vector<pcl::PointCloud<pcl::PointXYZI>>>&
           lidar_scans :
       lidar_data_) {
    if (use_n_frames_ > lidar_scans.second.size()) {
      return false;
    }
  }

  return true;
}

kindr::minimal::QuatTransformation LidarAligner::getTransformAB(int lidar_A_id, int lidar_B_id){
  return tforms_[lidar_A_id].inverse() * tforms_[lidar_B_id];
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "lidar_align");

  ros::NodeHandle nh, nh_private("~");

  std::string input_bag_path;
  if (!nh_private.getParam("input_bag_path", input_bag_path)) {
    ROS_FATAL("Could not find input_bag_path parameter, exiting");
    exit(EXIT_FAILURE);
  }

  LidarAligner* lidar_aligner = new LidarAligner(nh, nh_private);

  rosbag::Bag bag;
  bag.open(input_bag_path, rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TypeQuery("sensor_msgs/PointCloud2"));

  for (const rosbag::MessageInstance& m : view) {
    pcl::PointCloud<pcl::PointXYZI> pointcloud;
    pcl::fromROSMsg(*(m.instantiate<sensor_msgs::PointCloud2>()), pointcloud);

    lidar_aligner->addLidarScan(pointcloud, m.getTopic());

    if (lidar_aligner->haveEnoughScans()) {
      break;
    }
  }

  ceres::CostFunction* cost_function = new ceres::NumericDiffCostFunction<LidarAligner, ceres::CENTRAL, 1, 6>(
          lidar_aligner);
  ceres::Problem problem;

  kindr::minimal::QuatTransformation::Vector6 vec = kindr::minimal::QuatTransformation::log(lidar_aligner->getTransformAB(1,3));
  problem.AddResidualBlock(cost_function, NULL, vec.data());

  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;

  // Run the solver!
  ceres::Solver::Summary summary;
  Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";

  kindr::minimal::QuatTransformation tform = kindr::minimal::QuatTransformation::exp(kindr::minimal::QuatTransformation::Vector6(vec));
  ROS_ERROR_STREAM("Inital Tform " << vec);
  ROS_ERROR_STREAM("Final Tform " << tform.getTransformationMatrix());

  return 0;
}
