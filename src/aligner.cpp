#include "lidar_align/aligner.h"

Aligner::Aligner(const std::shared_ptr<Table>& table_ptr, const Config& config)
    : table_ptr_(table_ptr), config_(config){
    };

void Aligner::updateTableRow(const Lidar& lidar) {
  std::vector<double> vec =
      transformToVec(lidar.getOdomLidarTransform().inverse(), 6);
  table_ptr_->updateRow(lidar.getId(), vec);
}

void Aligner::updateTableFooter(const Scalar error) {
  std::stringstream ss;
  ss << "Total error         " << error;
  table_ptr_->updateFooter(ss.str());
}

Scalar Aligner::kNNError(const pcl::KdTreeFLANN<Point>& kdtree,
                         const Pointcloud& pointcloud, const size_t k,
                         const float max_dist, const size_t start_idx,
                         const size_t end_idx) {
  std::vector<int> kdtree_idx(k);
  std::vector<float> kdtree_dist(k);

  Scalar error = 0;
  for (size_t idx = start_idx; idx < std::min(pointcloud.size(), end_idx);
       ++idx) {
    kdtree.nearestKSearch(pointcloud[idx], k, kdtree_idx, kdtree_dist);
    for (const float& x : kdtree_dist) {
      error += std::min(x, max_dist);
    }
  }
  return error;
}

Scalar Aligner::lidarOdomKNNError(const Pointcloud& base_pointcloud,
                                  const Pointcloud& combined_pointcloud) const {
  // kill optimization if node stopped
  if (!ros::ok()) {
    throw std::runtime_error("ROS node died, exiting");
  }

  // shared_pointer needed by kdtree, no-op destructor to prevent it trying to
  // clean it up after use
  Pointcloud::ConstPtr combined_pointcloud_ptr(&combined_pointcloud,
                                               [](const Pointcloud*) {});

  pcl::KdTreeFLANN<Point> kdtree;

  kdtree.setInputCloud(combined_pointcloud_ptr);

  size_t k = config_.knn_k;
  // if searching own cloud add one to k as a point will always match to itself
  if (&base_pointcloud == &combined_pointcloud) {
    ++k;
  }

  // small amount of threading here to take edge off of bottleneck
  // break knn lookup up into several smaller problems each running in their own
  // thread
  std::vector<std::future<Scalar>> errors;
  for (size_t start_idx = 0; start_idx < base_pointcloud.size();
       start_idx += config_.knn_batch_size) {
    size_t end_idx =
        start_idx + std::min(base_pointcloud.size() - start_idx,
                             static_cast<size_t>(config_.knn_batch_size));
    errors.emplace_back(std::async(std::launch::async, Aligner::kNNError,
                                   kdtree, base_pointcloud, k,
                                   config_.knn_max_dist, start_idx, end_idx));
  }

  // wait for threads to finish and grab results
  Scalar total_error = 0;
  for (std::future<Scalar>& error : errors) {
    total_error += error.get();
  }

  return total_error;
}

Scalar Aligner::lidarOdomKNNError(const Lidar& lidar) const {
  Pointcloud pointcloud;
  lidar.getCombinedPointcloud(&pointcloud);
  return lidarOdomKNNError(pointcloud, pointcloud);
}

Scalar Aligner::lidarOdomKNNError(const LidarArray& lidar_array) const {
  Scalar total_error = 0;
  for (const Lidar& base_lidar : lidar_array.getLidarVector()) {
    Pointcloud base_pointcloud;
    base_lidar.getCombinedPointcloud(&base_pointcloud);

    for (const Lidar& lidar : lidar_array.getLidarVector()) {
      Pointcloud combined_pointcloud;
      lidar.getCombinedPointcloud(&combined_pointcloud);
      total_error += lidarOdomKNNError(combined_pointcloud, base_pointcloud);
    }
  }

  return total_error;
}

Transform Aligner::vecToTransform(const std::vector<double>& vec,
                                  const Transform& inital_T) {
  Transform::Vector6 tf_vec = inital_T.log();

  // 1 = rz
  // 2 = x,y
  // 3 = rx,ry,rz
  // 5 = x,y,rx,ry,rz
  // 6 = x,y,z,rx,ry,rz

  switch (vec.size()) {
    case 1:
      tf_vec[5] = vec[0];
      break;
    case 2:
      tf_vec[0] = vec[0];
      tf_vec[1] = vec[1];
      break;
    case 3:
      tf_vec[3] = vec[0];
      tf_vec[4] = vec[1];
      tf_vec[5] = vec[2];
      break;
    case 5:
      tf_vec[0] = vec[0];
      tf_vec[1] = vec[1];
      tf_vec[3] = vec[2];
      tf_vec[4] = vec[3];
      tf_vec[5] = vec[4];
      break;
    case 6:
      tf_vec[0] = vec[0];
      tf_vec[1] = vec[1];
      tf_vec[2] = vec[2];
      tf_vec[3] = vec[3];
      tf_vec[4] = vec[4];
      tf_vec[5] = vec[5];
      break;
    default:
      throw std::runtime_error("Vector must be of size 1, 2, 3, 5 or 6");
  }

  return Transform::exp(tf_vec);
}

std::vector<double> Aligner::transformToVec(const Transform& T,
                                            size_t vec_length) {
  Transform::Vector6 log_vec = T.log();
  std::vector<double> out_vec;

  // 1 = rz
  // 2 = x,y
  // 3 = rx,ry,rz
  // 5 = x,y,rx,ry,rz
  // 6 = x,y,z,rx,ry,rz

  switch (vec_length) {
    case 1:
      out_vec.push_back(log_vec[0]);
      break;
    case 2:
      out_vec.push_back(log_vec[0]);
      out_vec.push_back(log_vec[1]);
      break;
    case 3:
      out_vec.push_back(log_vec[3]);
      out_vec.push_back(log_vec[4]);
      out_vec.push_back(log_vec[5]);
      break;
    case 5:
      out_vec.push_back(log_vec[0]);
      out_vec.push_back(log_vec[1]);
      out_vec.push_back(log_vec[3]);
      out_vec.push_back(log_vec[4]);
      out_vec.push_back(log_vec[5]);
      break;
    case 6:
      out_vec.push_back(log_vec[0]);
      out_vec.push_back(log_vec[1]);
      out_vec.push_back(log_vec[2]);
      out_vec.push_back(log_vec[3]);
      out_vec.push_back(log_vec[4]);
      out_vec.push_back(log_vec[5]);
      break;
    default:
      throw std::runtime_error("Vector must be of size 1, 2, 3, 5 or 6");
  }
  return out_vec;
}

std::vector<double> Aligner::createRangeVec(const std::vector<double>& full_vec,
                                            size_t vec_length) {
  // 1 = rz
  // 2 = x,y
  // 3 = rx,ry,rz
  // 5 = x,y,rx,ry,rz
  // 6 = x,y,z,rx,ry,rz

  std::vector<double> out_vec;

  switch (vec_length) {
    case 1:
      out_vec.push_back(full_vec[0]);
      break;
    case 2:
      out_vec.push_back(full_vec[0]);
      out_vec.push_back(full_vec[1]);
      break;
    case 3:
      out_vec.push_back(full_vec[3]);
      out_vec.push_back(full_vec[4]);
      out_vec.push_back(full_vec[5]);
      break;
    case 5:
      out_vec.push_back(full_vec[0]);
      out_vec.push_back(full_vec[1]);
      out_vec.push_back(full_vec[3]);
      out_vec.push_back(full_vec[4]);
      out_vec.push_back(full_vec[5]);
      break;
    case 6:
      out_vec.push_back(full_vec[0]);
      out_vec.push_back(full_vec[1]);
      out_vec.push_back(full_vec[2]);
      out_vec.push_back(full_vec[3]);
      out_vec.push_back(full_vec[4]);
      out_vec.push_back(full_vec[5]);
      break;
    default:
      throw std::runtime_error("Vector must be of size 1, 2, 3, 5 or 6");
  }
  return out_vec;
}

double Aligner::LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad, void* f_data) {
  std::pair<Lidar*, Aligner*> data =
      *static_cast<std::pair<Lidar*, Aligner*>*>(f_data);

  if (!grad.empty()) {
    //  std::cerr << "error gradient in use" << std::endl;
  }

  Transform T = Aligner::vecToTransform(x, data.first->getOdomLidarTransform());
  data.first->setOdomLidarTransform(T);

  data.second->updateTableRow(*(data.first));

  double error = data.second->lidarOdomKNNError(*data.first);

  data.second->updateTableFooter(error);

  return error;
}

void Aligner::lidarOdomTransform(const size_t num_params, Lidar* lidar_ptr) {
  std::pair<Lidar*, Aligner*> data = std::make_pair(lidar_ptr, this);

  nlopt::opt opt;
  if (config_.local) {
    opt = nlopt::opt(nlopt::LN_BOBYQA, num_params);
  } else {
    opt = nlopt::opt(nlopt::GN_DIRECT_L, num_params);
  }

  // check range of 10 meters and all angles
  std::vector<double> range = createRangeVec(config_.range, num_params);

  std::vector<double> lb(num_params);
  std::vector<double> ub(num_params);
  for (size_t i = 0; i < num_params; ++i) {
    lb[i] = config_.inital_guess[i] - range[i];
    ub[i] = config_.inital_guess[i] + range[i];
  }

  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);

  // only doing a rough estimate so stop quickly
  opt.set_maxeval(config_.max_evals);
  opt.set_xtol_abs(config_.xtol);

  opt.set_min_objective(LidarOdomMinimizer, &data);

  double minf;
  std::vector<double> x = config_.inital_guess;
  LidarOdomMinimizer(x, x, &data);
  nlopt::result result = opt.optimize(x, minf);
  LidarOdomMinimizer(x, x, &data);
}
