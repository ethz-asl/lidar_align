#ifndef LIDAR_ALIGN_CALIB_LOADER_H_
#define LIDAR_ALIGN_CALIB_LOADER_H_

#include <string>

#include <lidar_align/transform.h>

namespace lidar_align {

class CalibLoader {
 public:
  CalibLoader() {}

  // The reconfigure callback
  void loadFromTxt(const std::string& path, Transform* transform,
                   double* time_offset) const;

 private:
};

}  // namespace lidar_align

#endif  // LIDAR_ALIGN_CALIB_LOADER_H_
