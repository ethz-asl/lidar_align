#ifndef LIDAR_ALIGN_ALIGNER_H_
#define LIDAR_ALIGN_ALIGNER_H_

class Aligner {

Scalar trimmedMeans(const std::vector<Scalar>& raw_error,
                             const Scalar& inlier_ratio);
Transform ScanScanICPTransform(const Scan& scan_a, const Scan& scan_b,
                                        const Transform& T_a_b_inital) const;

Scalar ScanScanCPError(const Scan& scan_a, const Scan& scan_b,
                                const Transform& T_a_b) const;

Scalar LidarOdomCPError(const Lidar& lidar);
}