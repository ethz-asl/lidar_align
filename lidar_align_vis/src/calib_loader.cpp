#include "lidar_align_vis/calib_loader.h"

#include <fstream>
#include <sstream>

namespace lidar_align {

void CalibLoader::loadFromTxt(const std::string& path, Transform* transform,
                              double* time_offset) const {
  // NOTE(alexmillane): This function is some hard coded hackery. But will work
  // for now.

  // Getting the calibration strings
  std::string translation_str;
  std::string rotation_str;
  std::string time_offset_str;
  std::string line;
  std::ifstream file_stream(path);
  if (file_stream.is_open()) {
    while (getline(file_stream, line)) {
      // std::cout << line << '\n';
      if (line.compare("Active Translation Vector (x,y,z) from the Pose Sensor "
                       "Frame to  the Lidar Frame:") == 0) {
        getline(file_stream, translation_str);
      } else if (line.compare("Active Hamiltonen Quaternion (w,x,y,z) the Pose "
                              "Sensor Frame to  the Lidar Frame:") == 0) {
        getline(file_stream, rotation_str);
      } else if (line.compare("Time offset that must be added to lidar "
                              "timestamps in seconds:") == 0) {
        getline(file_stream, time_offset_str);
      }
    }
    file_stream.close();
  }

  // String to matrix (translation)
  Transform::Translation translation;
  translation_str = translation_str.substr(1, translation_str.size() - 2);
  std::stringstream translation_ss(translation_str);
  std::string val_str;
  size_t idx = 0;
  while (getline(translation_ss, val_str, ',')) {
    float val = std::stof(val_str);
    translation[idx] = val;
    idx++;
  }

  // String to matrix (rotation)
  Transform::Rotation rotation;
  rotation_str = rotation_str.substr(1, rotation_str.size() - 2);
  std::stringstream rotation_ss(rotation_str);
  idx = 0;
  while (getline(rotation_ss, val_str, ',')) {
    float val = std::stof(val_str);
    if (idx == 0) {
      rotation.w() = val;
    } else {
      rotation.vec()[idx - 1] = val;
    }
    idx++;
  }

  // Transform
  *transform = Transform(translation, rotation);

  // Time offset
  *time_offset = std::stod(time_offset_str);
}

}  // namespace lidar_align