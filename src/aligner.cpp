#include "lidar_align/aligner.h"
#include "lidar_align/icp.h"

Scalar Aligner::trimmedMeans(const std::vector<Scalar>& raw_error,
                             const Scalar& inlier_ratio) const {
  if ((inlier_ratio <= 0) || (inlier_ratio > 1)) {
    throw std::runtime_error("Inlier ratio must be > 0 and <= 1");
  }

  std::vector<Scalar> sorted_error = raw_error;
  std::sort(sorted_error.begin(), sorted_error.end());
  return std::accumulate(
      sorted_error.begin(),
      sorted_error.begin() + std::ceil(inlier_ratio * sorted_error.size()),
      0.0f) / (std::ceil(inlier_ratio * sorted_error.size()));
}

void Aligner::setLidarTransforms(Lidar* lidar) {
  Transform T;
  lidar->setLidarLidarTransform(T, 0);
  for (size_t idx = 1; idx < lidar->getNumberOfScans(); ++idx) {
    T = T * scanScanICPTransform(lidar->getScan(idx - 1), lidar->getScan(idx));
    std::cout << T.log().transpose() << std::endl;
    lidar->setLidarLidarTransform(T, idx);
  }
}

Transform Aligner::scanScanICPTransform(const Scan& scan_a, const Scan& scan_b,
                                        const Transform& T_a_b_inital) const {
  ICP icp;
  Transform T_a_b = T_a_b_inital;

  icp.setTgtPoints(scan_b.getRawPointcloud());
  icp.setSrcPoints(scan_a.getRawPointcloud());
  icp.setCurrentTform(T_a_b_inital);

  if (icp.runICP(config_.icp_iterations, config_.icp_inlier_ratio)) {
    T_a_b = icp.getCurrentTform();
  }

  return T_a_b;
}

Scalar Aligner::scanScanCPError(const Scan& scan_a, const Scan& scan_b,
                                const Transform& T_a_b) const {
  pcl::KdTreeFLANN<Point> scan_a_kdtree;
  Pointcloud::Ptr scan_a_ptr =
      boost::make_shared<Pointcloud>(*(new Pointcloud));

  pcl::transformPointCloud(scan_a.getRawPointcloud(), *scan_a_ptr,
                           T_a_b.cast<float>().getTransformationMatrix());

  scan_a_kdtree.setInputCloud(scan_a_ptr);

  std::vector<int> kdtree_idx(1);
  std::vector<float> kdtree_dist(1);

  std::vector<Scalar> raw_error;
  for (Point point : scan_b.getRawPointcloud()) {
    scan_a_kdtree.nearestKSearch(point, 1, kdtree_idx, kdtree_dist);
    raw_error.push_back(kdtree_dist[0]);
  }

  return trimmedMeans(raw_error, config_.cp_inlier_ratio);
}

Scalar Aligner::lidarOdomCPError(const Lidar& lidar) const {
  std::vector<Scalar> raw_error;
  std::cerr << "tf: " << lidar.getOdomLidarTransform().log().transpose() << " err: ";
  for (size_t idx = 1; idx < lidar.getNumberOfScans(); ++idx) {
    Transform T_o_ltm1 = lidar.getScan(idx - 1).getOdomTransform() *
                         lidar.getOdomLidarTransform();
    Transform T_o_lt =
        lidar.getScan(idx).getOdomTransform() * lidar.getOdomLidarTransform();

    Transform T_ltm1_lt = T_o_ltm1.inverse() * T_o_lt;

    std::cerr << "tdiff: " << lidar.getScan(idx).getOdomTransform().log().transpose() << std::endl;

    raw_error.push_back(
        scanScanCPError(lidar.getScan(idx - 1), lidar.getScan(idx), T_ltm1_lt));
  }

  Scalar err = trimmedMeans(raw_error, config_.lidar_odom_cp_inlier_ratio);
  std::cerr << err << std::endl;
  return err;
}

Aligner::LidarOdomCPMinimizer::LidarOdomCPMinimizer(const Aligner* aligner_ptr,
                                                    Lidar* lidar_ptr)
    : lidar_ptr_(lidar_ptr), aligner_ptr_(aligner_ptr) {}

bool Aligner::LidarOdomCPMinimizer::operator()(
    const double* const tform_vec_raw, double* residual) const {
  Transform::Vector6 vec;
  for (size_t i = 0; i < 6; ++i) {
    vec[i] = 100000000*tform_vec_raw[i];
  }
  lidar_ptr_->setOdomLidarTransform(Transform::exp(vec));
  *residual = aligner_ptr_->lidarOdomCPError(*lidar_ptr_);
  //*residual = std::rand();
  return true;
}

void Aligner::lidarOdomCPTransform(Lidar* lidar_ptr) {
  Eigen::Matrix<double, 6, 1> tform_vec =
      lidar_ptr->getOdomLidarTransform().log().cast<double>();

  // Build the problem.
  ceres::Problem problem;

  ceres::CostFunction* cost_function =
      new ceres::NumericDiffCostFunction<LidarOdomCPMinimizer, ceres::CENTRAL,
                                         1, 6>(
          new LidarOdomCPMinimizer(this, lidar_ptr));
  problem.AddResidualBlock(cost_function, NULL, &(tform_vec[0]));

  // std::cerr << aligner_ptr_->lidarOdomCPError(*lidar_ptr_) << std::endl;

  std::cerr << "Initial transform:\n" << lidar_ptr->getOdomLidarTransform() << "\n";

  // Run the solver!
  ceres::Solver::Options options;
  //options.function_tolerance = 0;
  //options.parameter_tolerance = 0;
  //options.gradient_tolerance = 0;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  std::cerr << "Final transform:\n" << lidar_ptr->getOdomLidarTransform() << "\n";
}
