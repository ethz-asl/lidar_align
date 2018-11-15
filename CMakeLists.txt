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

# NLOPT is frustratingly inconsistent in the name of its cmake file so we use our own
list(APPEND CMAKE_FIND_ROOT_PATH ${CMAKE_SOURCE_DIR})
find_package(NLOPT REQUIRED)

include_directories(
  include
  ${NLOPT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sensors.cpp
  src/aligner.cpp  
  src/loader.cpp
)

catkin_package(
   INCLUDE_DIRS include/
   LIBRARIES ${PROJECT_NAME}
)

add_executable(lidar_align_node
  src/lidar_align_node.cpp
  
)
target_link_libraries(lidar_align_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${NLOPT_LIBRARIES})

install(DIRECTORY include/lidar_align
  DESTINATION include
  FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
)
