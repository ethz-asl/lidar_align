#include "lidar_align/aligner.h"
#include "lidar_align/icp.h"

Scalar Aligner::trimmedMeans(const std::vector<Scalar>& raw_error,
                             const Scalar& inlier_ratio) const {
  if ((inlier_ratio <= 0) || (inlier_ratio > 1)) {
    throw std::runtime_error("Inlier ratio must be > 0 and <= 1");
  }

  std::vector<Scalar> sorted_error = raw_error;
  std::sort(sorted_error.begin(), sorted_error.end());
  return std::accumulate(sorted_error.begin(),
                         sorted_error.begin() +
                             std::ceil(inlier_ratio * sorted_error.size()),
                         0.0f) /
         (std::ceil(inlier_ratio * sorted_error.size()));
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
  std::cerr << "tf: " << lidar.getOdomLidarTransform().log().transpose()
            << " err: ";
  for (size_t idx = 1; idx < lidar.getNumberOfScans(); ++idx) {
    Transform T_o_ltm1 = lidar.getScan(idx - 1).getOdomTransform() *
                         lidar.getOdomLidarTransform();
    Transform T_o_lt =
        lidar.getScan(idx).getOdomTransform() * lidar.getOdomLidarTransform();

    Transform T_ltm1_lt = T_o_ltm1.inverse() * T_o_lt;
    // if(idx == 1){
    //  std::cerr << "tdiff: " <<
    //  lidar.getScan(idx).getOdomTransform().log().transpose() << std::endl;
    //}
    raw_error.push_back(
        scanScanCPError(lidar.getScan(idx - 1), lidar.getScan(idx), T_ltm1_lt));
  }

  Scalar err = trimmedMeans(raw_error, config_.lidar_odom_cp_inlier_ratio);
  std::cerr << err << std::endl;
  return err;
}

Scalar Aligner::lidarOdomProjGridError(const Lidar& lidar) const {
  Scalar grid_res = 0.1;
  std::map<std::tuple<size_t, size_t, size_t>, size_t> proj_grid;
  for (size_t idx = 0; idx < lidar.getNumberOfScans(); ++idx) {
    Pointcloud tformed_scan = lidar.getScan(idx).getTimeAlignedPointcloud(
        lidar.getOdomLidarTransform());

    for (Point& point : tformed_scan) {
      std::tuple<size_t, size_t, size_t> loc =
          std::make_tuple(static_cast<size_t>(point.x / grid_res),
                          static_cast<size_t>(point.y / grid_res),
                          static_cast<size_t>(point.z / grid_res));
      if (proj_grid.count(loc) == 0) {
        proj_grid[loc] = 1;
      } else {
        proj_grid[loc]++;
      }
    }
  }

  Scalar total_error = 0;
  for (const std::pair<std::tuple<size_t, size_t, size_t>, size_t>& cell :
       proj_grid) {
    total_error -= static_cast<Scalar>(cell.second) *
                   std::log2(static_cast<Scalar>(cell.second));
  }
  // total_error = proj_grid.size();
  std::cerr << "err: " << std::setw(15) << total_error
            << "  tf: " << lidar.getOdomLidarTransform().log().transpose()
            << std::endl;

  return total_error;
}

Transform Aligner::rawVec6ToTransform(const double* vec6) {
  Transform::Vector6 tf_vec;
  for (size_t i = 0; i < 6; ++i) {
    tf_vec[i] = vec6[i];
  }
  return Transform::exp(tf_vec);
}

Transform Aligner::rawVec5ToTransform(const double* vec5) {
  Transform::Vector6 tf_vec;
  for (size_t i = 0; i < 6; ++i) {
    if (i < 2) {
      tf_vec[i] = vec5[i];
    } else if (i == 2) {
      tf_vec[i] = 0;
    } else {
      tf_vec[i] = vec5[i - 1];
    }
  }
  return Transform::exp(tf_vec);
}

Transform Aligner::rawVec3ToTransform(const double* vec3) {
  Transform::Vector6 tf_vec;
  for (size_t i = 0; i < 6; ++i) {
    if (i < 2) {
      tf_vec[i] = vec3[i];
    } else if (i != 5) {
      tf_vec[i] = 0;
    } else {
      tf_vec[5] = vec3[2];
    }
  }
  return Transform::exp(tf_vec);
}

Transform Aligner::vecToTransform(const std::vector<double>& vec) {
  switch (vec.size()) {
    case 3:
      return rawVec3ToTransform(vec.data());
      break;
    case 5:
      return rawVec5ToTransform(vec.data());
      break;
    case 6:
      return rawVec6ToTransform(vec.data());
    default:
      throw std::runtime_error("Transform vectors must be of size 3, 5 or 6");
  }
}

double Aligner::LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad, void* f_data) {
  std::pair<Lidar*, Aligner*> data =
      *static_cast<std::pair<Lidar*, Aligner*>*>(f_data);

  if (!grad.empty()) {
    std::cerr << "error gradient in use" << std::endl;
  }

  Transform T = Aligner::vecToTransform(x);
  data.first->setOdomLidarTransform(T);
  return data.second->lidarOdomProjGridError(*data.first);

  if (!grad.empty()) {
    std::cerr << "error gradient in use" << std::endl;
  }
}

void Aligner::lidarOdomTransform(Lidar* lidar_ptr) {
  std::pair<Lidar*, Aligner*> data = std::make_pair(lidar_ptr, this);

  std::cerr << "Initial transform:\n"
            << lidar_ptr->getOdomLidarTransform() << "\n";

  nlopt::opt opt(nlopt::LN_NELDERMEAD, 6);

  std::vector<double> lb(6);
  lb[0] = -6;
  lb[1] = -6;
  lb[2] = -1;
  lb[3] = -0.1;
  lb[4] = -0.1;
  lb[5] = -3.14159;
  opt.set_lower_bounds(lb);

  std::vector<double> ub(6);
  ub[0] = 6;
  ub[1] = 6;
  ub[2] = 1;
  ub[3] = 0.1;
  ub[4] = 0.1;
  ub[5] = 3.14159;
  opt.set_upper_bounds(ub);

  opt.set_min_objective(LidarOdomMinimizer, &data);

  std::vector<double> inital_guess(6);
  inital_guess[0] = 0;
  inital_guess[1] = 0;
  inital_guess[2] = 0;
  inital_guess[3] = 0;
  inital_guess[4] = 0;
  inital_guess[5] = 0;

  double minf;
  nlopt::result result = opt.optimize(inital_guess, minf);

  std::vector<double> termination_tolerance(6);
  termination_tolerance[0] = 0.01;
  termination_tolerance[1] = 0.01;
  termination_tolerance[2] = 0.01;
  termination_tolerance[3] = 0.001;
  termination_tolerance[4] = 0.001;
  termination_tolerance[5] = 0.001;
  opt.set_xtol_abs(termination_tolerance);

  std::cerr << "Final transform:\n"
            << lidar_ptr->getOdomLidarTransform() << "\n";
}
