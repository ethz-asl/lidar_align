#include "lidar_align/aligner.h"

Aligner::Aligner(const std::shared_ptr<Table>& table_ptr, const Config& config)
    : table_ptr_(table_ptr), config_(config){};

Aligner::Config Aligner::getConfig(ros::NodeHandle* nh) {
  Aligner::Config config;
  nh->param("local", config.local, config.local);
  nh->param("inital_guess", config.inital_guess, config.inital_guess);
  nh->param("range", config.range, config.range);
  nh->param("max_evals", config.max_evals, config.max_evals);
  nh->param("xtol", config.xtol, config.xtol);
  nh->param("knn_batch_size", config.knn_batch_size, config.knn_batch_size);
  nh->param("knn_k", config.knn_k, config.knn_k);
  nh->param("knn_max_dist", config.knn_max_dist, config.knn_max_dist);
  nh->param("time_cal", config.time_cal, config.time_cal);
  nh->param("output_pointcloud_path", config.output_pointcloud_path,
            config.output_pointcloud_path);
  nh->param("output_calibration_path", config.output_calibration_path,
            config.output_calibration_path);

  return config;
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

double Aligner::LidarOdomMinimizer(const std::vector<double>& x,
                                   std::vector<double>& grad, void* f_data) {
  OptData* d = static_cast<OptData*>(f_data);

  if (d->time_cal) {
    d->lidar->setOdomOdomTransforms(*(d->odom), x[6]);
  }

  const Eigen::Matrix<double, 6, 1> vec(x.data());
  d->lidar->setOdomLidarTransform(Transform::exp(vec.cast<Scalar>()));

  d->table->updateRow(d->lidar->getId(), x);

  double error = d->aligner->lidarOdomKNNError(*(d->lidar));

  static int i = 0;
  std::stringstream ss;
  ss << "Total error: " << error << " \tat iteration: " << i++;
  d->table->updateFooter(ss.str());

  return error;
}

void Aligner::lidarOdomTransform(Lidar* lidar, Odom* odom) {
  OptData opt_data;
  opt_data.lidar = lidar;
  opt_data.odom = odom;
  opt_data.table = table_ptr_;
  opt_data.aligner = this;
  opt_data.time_cal = config_.time_cal;

  size_t num_params = 6;
  if (config_.time_cal) {
    ++num_params;
  }

  nlopt::opt opt;
  if (config_.local) {
    opt = nlopt::opt(nlopt::LN_BOBYQA, num_params);
  } else {
    opt = nlopt::opt(nlopt::GN_DIRECT_L, num_params);
  }

  std::vector<double> lb(num_params);
  std::vector<double> ub(num_params);
  for (size_t i = 0; i < num_params; ++i) {
    lb[i] = config_.inital_guess[i] - config_.range[i];
    ub[i] = config_.inital_guess[i] + config_.range[i];
  }

  opt.set_lower_bounds(lb);
  opt.set_upper_bounds(ub);

  // only doing a rough estimate so stop quickly
  opt.set_maxeval(config_.max_evals);
  opt.set_xtol_abs(config_.xtol);

  opt.set_min_objective(LidarOdomMinimizer, &opt_data);

  double minf;
  std::vector<double> x = config_.inital_guess;
  std::vector<double> grad;
  nlopt::result result = opt.optimize(x, minf);
  LidarOdomMinimizer(x, grad, &opt_data);

  if (!config_.output_pointcloud_path.empty()) {
    table_ptr_->updateHeader("Saving calibration pointcloud");
    lidar->saveCombinedPointcloud(config_.output_pointcloud_path);
  }

  if (!config_.output_calibration_path.empty()) {
    table_ptr_->updateHeader("Saving calibration file");
    std::ofstream file;
    file.open(config_.output_calibration_path, std::ofstream::out);
    file << "T Vector: ";
    for (double value : x) {
      file << value << " ";
    }
    file << std::endl;
    file << "T: " << std::endl;
    file << lidar->getOdomLidarTransform() << std::endl;
    file.close();
  }
}
