#include "lidar_align/aligner.h"
#include "lidar_align/icp.h"

Scalar Aligner::trimmedMeans(const std::vector<Scalar>& raw_error,
                             const Scalar& inlier_ratio) {
  if((inlier_ratio <= 0) || (inlier_ratio > 1){
    throw std::runtime_error("Inlier ratio must be > 0 and <= 1");
  }

  std::sort(raw_error.begin(), raw_error.end());
  return std::accumulate(
      raw_error.begin(),
      raw_error.begin() + std::ceil(inlier_ratio * raw_error.size()), 0.0f);
}

Transform Aligner::ScanScanICPTransform(const Scan& scan_a, const Scan& scan_b,
                                        const Transform& T_a_b_inital) const {
  ICP icp;
  Transform T_a_b = T_a_b_inital;

  icp.setTgtPoints(scan_b.getRawPointcloud());
  icp.setSrcPoints(scan_a.getRawPointcloud());
  icp.setCurrentTform(T_a_b_inital);

  if (icp.runICP(icp_iterations_, icp_inlier_ratio_)) {
    T_a_b = icp.getCurrentTform();
  }

  return T_At_Atp1;
}

Scalar Aligner::ScanScanCPError(const Scan& scan_a, const Scan& scan_b,
                                const Transform& T_a_b) const {
  pcl::KdTreeFLANN<Point> scan_a_kdtree;
  Pointcloud::Ptr scan_a_ptr =
      boost::make_shared<Pointcloud>(*(new Pointcloud));

  pcl::transformPointCloud(scan_a.getRawPointcloud(), *scan_a_ptr,
                           T_a_b.cast<float>().getTransformationMatrix());

  scan_At_kdtree.setInputCloud(scan_a_ptr);

  std::vector<int> kdtree_idx(1);
  std::vector<float> kdtree_dist(1);

  std::vector<Scalar> raw_error;
  for (Point point : scan_b.getRawPointcloud()) {
    scan_a_kdtree.nearestKSearch(point, 1, kdtree_idx, kdtree_dist);
    raw_error.push_back(kdtree_dist[0]);
  }

  return trimmedMeans(raw_error, cp_inlier_ratio) 
}

Scalar Aligner::LidarOdomCPError(const Lidar& lidar) {

  std::vector<Scalar> raw_error;
  for (size_t idx = 1; idx < lidar.getNumberOfScans(); ++idx) {
    Transform T_o_ltm1 = lidar.getScan(idx - 1).getOdomTransform() * T_o_l_;
    Transform T_o_lt = lidar.getScan(idx).getOdomTransform() * T_o_l_;

    Transform T_ltm1_lt = T_o_ltm1.inverse() * T_o_lt;

    error.push_back(aligner.ScanScanCPError(lidar.getScan(idx-1), lidar.getScan(idx), T_ltm1_lt);
  }

  return trimmedMeans(raw_error, lidar_odom_cp_inlier_ratio);
}