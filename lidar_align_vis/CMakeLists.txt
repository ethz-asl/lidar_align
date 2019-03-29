cmake_minimum_required(VERSION 2.8.0)
project(lidar_align_vis)

add_definitions(-std=c++11 -march=native)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Mark these as system deps to supress warnings.
#include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
cs_add_library(lidar_align_vis
  src/lidar_align_server.cpp
  src/calib_loader.cpp
)
target_link_libraries(lidar_align_vis ${catkin_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(lidar_align_vis_node
 src/lidar_align_vis_node.cpp
)
target_link_libraries(lidar_align_vis_node lidar_align_vis ${catkin_LIBRARIES})

# Make sure configure headers are built before any node using them
# Note(alexmillane): This is done automatically by catkin simple.
add_dependencies(lidar_align_vis_node ${PROJECT_NAME}_gencfg)

##########
# EXPORT #
##########

cs_install()
cs_export()