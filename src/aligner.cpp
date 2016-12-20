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

Scalar Aligner::thresholdedSum(const std::vector<Scalar>& raw_error,
                               const Scalar& threshold) const {
  Scalar total;
  for (const Scalar& x : raw_error) {
    total += std::min(x, threshold);
  }
  return total;
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
  // std::cerr << "tf: " << lidar.getOdomLidarTransform().log().transpose()
  //          << " err: ";
  for (size_t idx = 1; idx < lidar.getNumberOfScans(); ++idx) {
    Transform T_o_ltm1 = lidar.getScan(idx - 1).getOdomTransform() *
                         lidar.getOdomLidarTransform();
    Transform T_o_lt =
        lidar.getScan(idx).getOdomTransform() * lidar.getOdomLidarTransform();

    Transform T_ltm1_lt = T_o_ltm1.inverse() * T_o_lt;

    raw_error.push_back(
        scanScanCPError(lidar.getScan(idx - 1), lidar.getScan(idx), T_ltm1_lt));
  }

  Scalar err = trimmedMeans(raw_error, config_.lidar_odom_cp_inlier_ratio);
  return err;
}

Scalar Aligner::lidarOdomKNNError(const Lidar& lidar, const size_t k) const {
  pcl::KdTreeFLANN<Point> kdtree;
  Pointcloud::Ptr combined_ptr =
      boost::make_shared<Pointcloud>(*(new Pointcloud));

  for (size_t idx = 0; idx < lidar.getNumberOfScans(); ++idx) {
    Pointcloud tformed_scan = lidar.getScan(idx).getTimeAlignedPointcloud(
        lidar.getOdomLidarTransform());

    for (Point& point : tformed_scan) {
      combined_ptr->push_back(point);
    }
  }

  kdtree.setInputCloud(combined_ptr);

  std::vector<int> kdtree_idx(k);
  std::vector<float> kdtree_dist(k);

  Scalar error = 0;
  for (Point point : *combined_ptr) {
    kdtree.nearestKSearch(point, k, kdtree_idx, kdtree_dist);
    for (const Scalar& x : kdtree_dist) {
      error += std::min(x,1.0);
    }
  }

  return error;
}

Scalar Aligner::lidarsOdomKNNError(Lidars& lidars, const size_t k) {
  pcl::KdTreeFLANN<Point> kdtree;
  Pointcloud::Ptr combined_ptr =
      boost::make_shared<Pointcloud>(*(new Pointcloud));

  for(Lidar& lidar : lidars.getLidarsRef()){ 
    for (size_t idx = 0; idx < lidar.getNumberOfScans(); ++idx) {
      Pointcloud tformed_scan = lidar.getScan(idx).getTimeAlignedPointcloud(
          lidar.getOdomLidarTransform());

      for (Point& point : tformed_scan) {
        combined_ptr->push_back(point);
      }
    }
  }

  kdtree.setInputCloud(combined_ptr);

  std::vector<int> kdtree_idx(k);
  std::vector<float> kdtree_dist(k);

  Scalar error = 0;
  for (Point point : *combined_ptr) {
    kdtree.nearestKSearch(point, k, kdtree_idx, kdtree_dist);
    for (const Scalar& x : kdtree_dist) {
      error += std::min(x,1.0);
    }
  }

  return error;
}

Scalar Aligner::lidarsOdomVoxelEntropyError(Lidars& lidars) const {
  VoxelPyramid voxel_pyramid(0.01, 10);
  voxel_pyramid.addLidars(lidars);
  return voxel_pyramid.calculateEntropy();
}

Scalar Aligner::lidarOdomVoxelEntropyError(Lidar& lidar) const {
  VoxelPyramid voxel_pyramid(0.01, 10);
  voxel_pyramid.addLidar(lidar);
  return voxel_pyramid.calculateEntropy();

  Scalar error = 0;
  for (size_t idx = 1; idx < lidar.getNumberOfScans(); ++idx) {
    VoxelPyramid voxel_pyramid(0.01, 10);

    voxel_pyramid.addPointcloud(lidar.getScan(idx - 1).getTimeAlignedPointcloud(
        lidar.getOdomLidarTransform()));
    voxel_pyramid.addPointcloud(lidar.getScan(idx).getTimeAlignedPointcloud(
        lidar.getOdomLidarTransform()));
    error += voxel_pyramid.calculateEntropy();
  }
  return error;
}

Transform Aligner::vecToTransform(const std::vector<double>& vec,
                                  const Transform& inital_T) {
  Transform::Vector6 tf_vec = inital_T.log();
  size_t i = 0;
  if (vec.size() > 1) {
    tf_vec[0] = vec[i++];
    tf_vec[1] = vec[i++];
  }
  if (vec.size() == 6) {
    tf_vec[2] = vec[i++];
  }
  if (vec.size() > 3) {
    tf_vec[3] = vec[i++];
    tf_vec[4] = vec[i++];
  }
  if (vec.size() != 2) {
    tf_vec[5] = vec[i++];
  }
  return Transform::exp(tf_vec);
}

// 1 = rz
// 2 = x,y
// 3 = x,y,rz
// 5 = x,y,rx,ry,rz
// 6 = x,y,z,rx,ry,rz
std::vector<double> Aligner::transformToVec(const Transform& T,
                                            size_t vec_length) {
  Transform::Vector6 log_vec = T.log();
  std::vector<double> out_vec;

  if (vec_length > 1) {
    out_vec.push_back(log_vec[0]);
    out_vec.push_back(log_vec[1]);
  }
  if (vec_length == 6) {
    out_vec.push_back(log_vec[2]);
  }
  if (vec_length > 3) {
    out_vec.push_back(log_vec[3]);
    out_vec.push_back(log_vec[4]);
  }
  if (vec_length != 2) {
    out_vec.push_back(log_vec[5]);
  }
  return out_vec;
}

double Aligner::LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad, void* f_data) {
  std::pair<Lidar*, Aligner*> data =
      *static_cast<std::pair<Lidar*, Aligner*>*>(f_data);

  if (!grad.empty()) {
    std::cerr << "error gradient in use" << std::endl;
  }

  Transform T = Aligner::vecToTransform(x, data.first->getOdomLidarTransform());
  data.first->setOdomLidarTransform(T);

  double error = data.second->lidarOdomKNNError(*data.first,10);

  std::cout << "\b\r";
  std::cout << "Transform: ";
  for (size_t i = 0; i < x.size(); ++i) {
    std::cout << std::setw(12) << x[i] << " ";
  }
  std::cout << " Error: " << std::setw(12) << error << std::endl;

  return error;
}

double Aligner::LidarOdomJointMinimizer(const std::vector<double>& x,
                                        std::vector<double>& grad,
                                        void* f_data) {
  std::pair<Lidars*, Aligner*> data =
      *static_cast<std::pair<Lidars*, Aligner*>*>(f_data);

  if (!grad.empty()) {
    std::cerr << "error gradient in use" << std::endl;
  }

  std::vector<Lidar>& lidar_vec = data.first->getLidarsRef();

  for (size_t i = 0; i < (lidar_vec.size() + 1); ++i) {
    std::cout << "\b\r";
  }

  size_t part_len = x.size() / lidar_vec.size();
  std::vector<double>::const_iterator vec_start = x.begin();
  for (Lidar& lidar : lidar_vec) {
    std::vector<double> x_part(vec_start, vec_start + part_len);
    Transform T =
        Aligner::vecToTransform(x_part, lidar.getOdomLidarTransform());
    lidar.setOdomLidarTransform(T);
    vec_start += part_len;

    std::cout << "Lidar: " << lidar.getId() << " ";
    std::cout << "Transform: ";
    for (size_t i = 0; i < x_part.size(); ++i) {
      std::cout << std::setw(12) << x_part[i] << " ";
    }
    std::cout << std::endl;
  }

  double error = data.second->lidarsOdomKNNError(*data.first,10);

  std::cout << "Error: " << std::setw(12) << error << std::endl;

  return error;
}

void Aligner::lidarOdomTransform(const size_t num_params, Lidar* lidar_ptr) {
  std::pair<Lidar*, Aligner*> data = std::make_pair(lidar_ptr, this);

  std::cerr << "Initial transform:\n"
            << lidar_ptr->getOdomLidarTransform() << "\n";

  nlopt::opt opt(nlopt::GN_DIRECT_L, num_params);

  std::vector<double> inital_guess =
      transformToVec(lidar_ptr->getOdomLidarTransform(), num_params);

  std::vector<double> range;
  if (num_params > 1) {
    range.push_back(4);
    range.push_back(1.5);
  }
  if (num_params == 6) {
    range.push_back(0.5);
  }
  if (num_params > 3) {
    range.push_back(0.1);
    range.push_back(0.1);
  }
  if (num_params == 1) {
    range.push_back(3.2);
  } else if (num_params >= 3) {
    range.push_back(0.5);
  }

  std::vector<double> lb(num_params);
  std::vector<double> ub(num_params);
  for (size_t i = 0; i < num_params; ++i) {
    lb[i] = inital_guess[i] - range[i];
    ub[i] = inital_guess[i] + range[i];
  }

  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_maxeval(300);

  opt.set_min_objective(LidarOdomMinimizer, &data);

  double minf;
  nlopt::result result = opt.optimize(inital_guess, minf);
  LidarOdomMinimizer(inital_guess, inital_guess, &data);

  std::cerr << "Final transform:\n"
            << lidar_ptr->getOdomLidarTransform() << "\n";
}

void Aligner::lidarOdomJointTransform(const size_t num_params,
                                      Lidars* lidars_ptr) {
  std::pair<Lidars*, Aligner*> data = std::make_pair(lidars_ptr, this);

  size_t offset = 0;
  const size_t num_lidar = lidars_ptr->getNumberOfLidars();

  std::vector<double> lb(num_lidar * num_params);
  std::vector<double> ub(num_lidar * num_params);
  std::vector<double> inital_guess(num_lidar * num_params);

  for (size_t i = 0; i < num_lidar; ++i) {
    std::vector<double> temp = transformToVec(
        lidars_ptr->getLidarsRef()[i].getOdomLidarTransform(), num_params);
    for (size_t j = 0; j < temp.size(); ++j) {
      inital_guess[offset + j] = temp[j];
    }

    std::vector<double> range;
    if (num_params > 1) {
      range.push_back(2);
      range.push_back(2);
    }
    if (num_params == 6) {
      range.push_back(0.5);
    }
    if (num_params > 3) {
      range.push_back(0.1);
      range.push_back(0.05);
    }
    range.push_back(0.05);

    for (size_t j = 0; j < num_params; ++j) {
      lb[offset + j] = inital_guess[offset + j] - range[j];
      ub[offset + j] = inital_guess[offset + j] + range[j];
    }

    offset += num_params;
  }

  nlopt::opt opt(nlopt::LN_NELDERMEAD, num_lidar * num_params);
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_maxeval(4000);
  opt.set_min_objective(LidarOdomJointMinimizer, &data);
  double minf;
  nlopt::result result = opt.optimize(inital_guess, minf);
  LidarOdomJointMinimizer(inital_guess, inital_guess, &data);


  // std::cerr << "Final transform:\n"
  //          << lidar_ptr->getOdomLidarTransform() << "\n";
}

VoxelPyramid::VoxelPyramid(float element_size, size_t levels)
    : element_size_(element_size), voxel_pyramid_(levels){};

void VoxelPyramid::addPointcloud(const Pointcloud& pointcloud) {
  for (const Point& point : pointcloud) {
    std::tuple<int, int, int> voxel_location =
        std::make_tuple(static_cast<int>(point.x / element_size_),
                        static_cast<int>(point.y / element_size_),
                        static_cast<int>(point.z / element_size_));

    for (std::map<std::tuple<int, int, int>, int>& voxel_map : voxel_pyramid_) {
      voxel_map[voxel_location]++;
      voxel_location = std::make_tuple(std::get<0>(voxel_location) / 2,
                                       std::get<1>(voxel_location) / 2,
                                       std::get<2>(voxel_location) / 2);
    }
  }
}

void VoxelPyramid::addLidar(Lidar& lidar) {
  for (size_t idx = 0; idx < lidar.getNumberOfScans(); ++idx) {
    addPointcloud(lidar.getScan(idx).getTimeAlignedPointcloud(
        lidar.getOdomLidarTransform()));
  }
}

void VoxelPyramid::addLidars(Lidars& lidars) {
  for (Lidar& lidar : lidars.getLidarsRef()) {
    addLidar(lidar);
  }
}

Scalar VoxelPyramid::calculateEntropy() {
  Scalar entropy = 0;

  for (std::map<std::tuple<int, int, int>, int>& voxel_map : voxel_pyramid_) {
    // entropy += voxel_map.size();/*
    for (const std::pair<std::tuple<int, int, int>, int>& voxel : voxel_map) {
      entropy += std::log2(static_cast<Scalar>(voxel.second));
    }
    // entropy /= 2;
  }
  return entropy;
}

void VoxelPyramid::clear() {
  for (std::map<std::tuple<int, int, int>, int>& voxel_map : voxel_pyramid_) {
    voxel_map.clear();
  }
}