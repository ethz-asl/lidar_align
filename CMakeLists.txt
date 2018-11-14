cmake_minimum_required(VERSION 2.8.3)
project(lidar_align)

add_definitions(-std=c++11 -ofast)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  image_transport
  cv_bridge
  pcl_ros
  rosbag
  sensor_msgs
  geometry_msgs
)

find_package(nlopt REQUIRED)

include_directories(
  include
  ${nlopt_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_align_node
  src/lidar_align_node.cpp
  src/sensors.cpp
  src/aligner.cpp  
  src/loader.cpp
)
target_link_libraries(lidar_align_node ${catkin_LIBRARIES} ${nlopt_LIBRARIES})
