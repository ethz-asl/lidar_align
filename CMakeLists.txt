cmake_minimum_required(VERSION 2.8.3)
project(lidar_align)

add_definitions(-std=c++11 -ofast)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(nlopt REQUIRED)

include_directories(
  ${nlopt_INCLUDE_DIRS}
)

############
# BINARIES #
############
add_executable(lidar_align 
  src/lidar_align_node.cpp
  src/sensors.cpp
  src/aligner.cpp  
  src/loader.cpp
)
target_link_libraries(lidar_align ${catkin_LIBRARIES} ${nlopt_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
