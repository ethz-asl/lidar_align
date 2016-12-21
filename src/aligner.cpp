#include "lidar_align/aligner.h"

Aligner::Aligner(std::shared_ptr<Table> table_ptr) : table_ptr_(table_ptr){};

void Aligner::updateTableRow(const Lidar& lidar) {
  std::vector<double> vec = transformToVec(lidar.getOdomLidarTransform(), 6);
  table_ptr_->updateRow(lidar.getId(), vec);
}

void Aligner::updateTableFooter(const Scalar error) {
  std::stringstream ss;
  ss << "Total error         " << error;
  table_ptr_->updateFooter(ss.str());
}

Scalar Aligner::kNNError(const pcl::KdTreeFLANN<Point>& kdtree,
                         const Pointcloud& pointcloud, const size_t k,
                         const float max_distance, const size_t start_idx,
                         const size_t end_idx) {
  std::vector<int> kdtree_idx(k);
  std::vector<float> kdtree_dist(k);

  Scalar error = 0;
  for (size_t idx = start_idx; idx < std::min(pointcloud.size(), end_idx);
       ++idx) {
    kdtree.nearestKSearch(pointcloud[idx], k, kdtree_idx, kdtree_dist);
    for (const float& x : kdtree_dist) {
      //error += std::log(x + 1);
      error += std::min(x, max_distance);
    }
  }
  return error;
}

Scalar Aligner::lidarOdomKNNError(const Pointcloud& pointcloud) const {
  // kill optimization if node stopped
  if (!ros::ok()) {
    throw std::runtime_error("ROS node died, exiting");
  }

  // shared_pointer needed by kdtree, no-op destructor to prevent it trying to
  // clean it up after use
  Pointcloud::ConstPtr pointcloud_ptr(&pointcloud, [](const Pointcloud*) {});

  pcl::KdTreeFLANN<Point> kdtree;

  kdtree.setInputCloud(pointcloud_ptr);

  // small amount of threading here to take edge off of bottleneck
  // break knn lookup up into several smaller problems each running in their own
  // thread
  std::vector<std::future<Scalar>> errors;
  for (size_t start_idx = 0; start_idx < pointcloud.size();
       start_idx += config_.knn_batch_size) {
    size_t end_idx = start_idx + std::min(pointcloud.size() - start_idx,
                                          config_.knn_batch_size);
    errors.emplace_back(std::async(std::launch::async, Aligner::kNNError,
                                   kdtree, pointcloud, config_.knn_k+1,
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
  return lidarOdomKNNError(pointcloud);
}

Scalar Aligner::lidarOdomKNNError(const LidarArray& lidar_array) const {
  Scalar total_error = 0;

  for (const Lidar& lidar : lidar_array.getLidarVector()) {
    Pointcloud pointcloud;
    Pointcloud base;
    for (const Lidar& lidar2 : lidar_array.getLidarVector()) {
      if (lidar.getId() != lidar2.getId()) {
        lidar2.getCombinedPointcloud(&pointcloud);
      } else {
        lidar2.getCombinedPointcloud(&base);
      }
    }

    // kill optimization if node stopped
    if (!ros::ok()) {
      throw std::runtime_error("ROS node died, exiting");
    }

    // shared_pointer needed by kdtree, no-op destructor to prevent it trying to
    // clean it up after use
    Pointcloud::ConstPtr pointcloud_ptr(&pointcloud, [](const Pointcloud*) {});

    pcl::KdTreeFLANN<Point> kdtree;

    kdtree.setInputCloud(pointcloud_ptr);

    // small amount of threading here to take edge off of bottleneck
    // break knn lookup up into several smaller problems each running in their
    // own
    // thread
    std::vector<std::future<Scalar>> errors;
    for (size_t start_idx = 0; start_idx < base.size();
         start_idx += config_.knn_batch_size) {
      size_t end_idx =
          start_idx + std::min(base.size() - start_idx, config_.knn_batch_size);
      errors.emplace_back(std::async(std::launch::async, Aligner::kNNError,
                                     kdtree, base, config_.knn_k,
                                     config_.knn_max_dist, start_idx, end_idx));
    }

    // wait for threads to finish and grab results
    for (std::future<Scalar>& error : errors) {
      total_error += error.get();
    }
  }

  return total_error;
}

Transform Aligner::vecToTransform(const std::vector<double>& vec,
                                  const Transform& inital_T) {
  Transform::Vector6 tf_vec = inital_T.log();
  size_t i = 0;

  // 1 = rz
  // 2 = x,y
  // 3 = x,y,rz
  // 5 = x,y,rx,ry,rz
  // 6 = x,y,z,rx,ry,rz

  if (vec.size() > 6) {
    throw std::runtime_error("Vector must be of size 6 or less");
  }
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

std::vector<double> Aligner::transformToVec(const Transform& T,
                                            size_t vec_length) {
  Transform::Vector6 log_vec = T.log();
  std::vector<double> out_vec;

  // 1 = rz
  // 2 = x,y
  // 3 = x,y,rz
  // 5 = x,y,rx,ry,rz
  // 6 = x,y,z,rx,ry,rz

  if (vec_length > 6) {
    throw std::runtime_error("Vector must be of size 6 or less");
  }
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
    //  std::cerr << "error gradient in use" << std::endl;
  }

  Transform T = Aligner::vecToTransform(x, data.first->getOdomLidarTransform());
  data.first->setOdomLidarTransform(T);

  data.second->updateTableRow(*(data.first));

  double error = data.second->lidarOdomKNNError(*data.first);

  data.second->updateTableFooter(error);

  return error;
}

double Aligner::LidarOdomJointMinimizer(const std::vector<double>& x,
                                        std::vector<double>& grad,
                                        void* f_data) {
  std::pair<LidarArray*, Aligner*> data =
      *static_cast<std::pair<LidarArray*, Aligner*>*>(f_data);

  if (!grad.empty()) {
    //  std::cerr << "error gradient in use" << std::endl;
  }

  std::vector<Lidar>& lidar_vec = data.first->getLidarVector();

  size_t part_len = x.size() / lidar_vec.size();
  std::vector<double>::const_iterator vec_start = x.begin();
  for (Lidar& lidar : lidar_vec) {
    std::vector<double> x_part(vec_start, vec_start + part_len);
    Transform T =
        Aligner::vecToTransform(x_part, lidar.getOdomLidarTransform());
    lidar.setOdomLidarTransform(T);
    data.second->updateTableRow(lidar);

    vec_start += part_len;
  }

  double error = data.second->lidarOdomKNNError(*data.first);

  data.second->updateTableFooter(error);

  return error;
}

void Aligner::lidarOdomTransform(const size_t num_params, Lidar* lidar_ptr) {
  std::pair<Lidar*, Aligner*> data = std::make_pair(lidar_ptr, this);

  nlopt::opt opt(nlopt::GN_DIRECT_L, num_params);

  std::vector<double> inital_guess =
      transformToVec(lidar_ptr->getOdomLidarTransform(), num_params);

  // check range of 10 meters + within 10 degrees of horizontal
  std::vector<double> range;
  if (num_params > 1) {
    range.push_back(10);
    range.push_back(10);
  }
  if (num_params == 6) {
    range.push_back(10);
  }
  if (num_params > 3) {
    range.push_back(0.17);
    range.push_back(0.17);
  }
  if (num_params != 2) {
    range.push_back(3.2);
  }

  std::vector<double> lb(num_params);
  std::vector<double> ub(num_params);
  for (size_t i = 0; i < num_params; ++i) {
    lb[i] = inital_guess[i] - range[i];
    ub[i] = inital_guess[i] + range[i];
  }

  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);

  // only doing a rough estimate so stop quickly
  opt.set_maxeval(200);
  opt.set_xtol_abs(0.05);

  opt.set_min_objective(LidarOdomMinimizer, &data);

  double minf;
  LidarOdomMinimizer(inital_guess, inital_guess, &data);
  nlopt::result result = opt.optimize(inital_guess, minf);
  LidarOdomMinimizer(inital_guess, inital_guess, &data);
}

void Aligner::lidarOdomJointTransform(const size_t num_params,
                                      LidarArray* lidar_array_ptr) {
  std::pair<LidarArray*, Aligner*> data = std::make_pair(lidar_array_ptr, this);

  size_t offset = 0;
  const size_t num_lidar = lidar_array_ptr->getNumberOfLidars();

  std::vector<double> lb(num_lidar * num_params);
  std::vector<double> ub(num_lidar * num_params);
  std::vector<double> inital_guess(num_lidar * num_params);

  for (size_t i = 0; i < num_lidar; ++i) {
    std::vector<double> temp = transformToVec(
        lidar_array_ptr->getLidarVector()[i].getOdomLidarTransform(),
        num_params);
    for (size_t j = 0; j < temp.size(); ++j) {
      inital_guess[offset + j] = temp[j];
    }

    std::vector<double> range;
    if (num_params > 1) {
      range.push_back(2);
      range.push_back(2);
    }
    if (num_params == 6) {
      range.push_back(1);
    }
    if (num_params > 3) {
      range.push_back(0.1);
      range.push_back(0.1);
    }
    if (num_params != 2) {
      range.push_back(0.2);
    }

    for (size_t j = 0; j < num_params; ++j) {
      lb[offset + j] = inital_guess[offset + j] - range[j];
      ub[offset + j] = inital_guess[offset + j] + range[j];
    }

    offset += num_params;
  }

  nlopt::opt opt(nlopt::LN_NELDERMEAD, num_lidar * num_params);
  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);
  opt.set_maxeval(2000);
  opt.set_xtol_abs(0.001);
  opt.set_min_objective(LidarOdomJointMinimizer, &data);
  double minf;
  nlopt::result result = opt.optimize(inital_guess, minf);
  LidarOdomJointMinimizer(inital_guess, inital_guess, &data);
}