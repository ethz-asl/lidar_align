cmake_minimum_required(VERSION 2.8.3)
project(lidar_align)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# BINARIES #
############
add_executable(lidar_align src/lidar_align.cpp)
target_link_libraries(lidar_align ${catkin_LIBRARIES})

##########
# EXPORT #
##########
cs_install()
cs_export()
