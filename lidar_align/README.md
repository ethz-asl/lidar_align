# lidar_align

## A simple method for finding the extrinsic calibration between a 3D lidar and a 6-dof pose sensor

**Note: Accurate results require highly non-planar motions, this makes the technique poorly suited for calibrating sensors mounted to cars.**

The method makes use of the property that pointclouds from lidars appear more 'crisp' when the calibration is correct. It does this as follows:
1) A transformation between the lidar and pose sensor is set.
2) The poses are used in combination with the above transformation to fuse all the lidar points into a single pointcloud.
3) The sum of the distance between each point and its nearest neighbor is found.
This process is repeated in an optimization that attempts to find the transformation that minimizes this distance.

## Installation

To install lidar_align, please install [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu), [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) or [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu).

The following additional system dependencies are also required:
```
sudo apt-get install libnlopt-dev
```
## Input Transformations

The final calibrations quality is strongly correlated with the quality of the transformation source and the range of motion observed. To ensure an accurate calibration the dataset should encompass a large range of rotations and translations. Motion that is approximately planner (for example a car driving down a street) does not provide any information about the system in the direction perpendicular to the plane, which will cause the optimizer to give incorrect estimates in this direction.

## Estimation proceedure
For most systems the node can be run without tuning the parameters. By default two optimizations are performed, a rough angle only global optimization followed by a local 6-dof refinement.

The node will load all messages of type `sensor_msgs/PointCloud2` from the given ROS bag for use as the lidar scans to process. The poses can either be given in the same bag file as `geometry_msgs/TransformStamped` messages or in a separate CSV file that follows the format of [Maplab](https://github.com/ethz-asl/maplab).

## Visualization and Results

The node will output it's current estimated transform while running. To view this your launchfile must set `output="screen"` in the `<node/>` section. See the given launchfile for an example.

Once the optimization finishes the transformation parameters will be printed to the console. An example output is as follows:
```
Active Transformation Vector (x,y,z,rx,ry,rz) from the Pose Sensor Frame to  the Lidar Frame:
[-0.0608575, -0.0758112, 0.27089, 0.00371254, 0.00872398, 1.60227]

Active Transformation Matrix from the Pose Sensor Frame to  the Lidar Frame:
-0.0314953  -0.999473  0.0078319 -0.0608575
  0.999499 -0.0314702 0.00330021 -0.0758112
 -0.003052 0.00793192   0.999964    0.27089
         0          0          0          1

Active Translation Vector (x,y,z) from the Pose Sensor Frame to  the Lidar Frame:
[-0.0608575, -0.0758112, 0.27089]

Active Hamiltonen Quaternion (w,x,y,z) the Pose Sensor Frame to  the Lidar Frame:
[0.69588, 0.00166397, 0.00391012, 0.718145]

Time offset that must be added to lidar timestamps in seconds:
0.00594481

ROS Static TF Publisher: <node pkg="tf" type="static_transform_publisher" name="pose_lidar_broadcaster" args="-0.0608575 -0.0758112 0.27089 0.00166397 0.00391012 0.718145 0.69588 POSE_FRAME LIDAR_FRAME 100" />
```
If the path has been set the results will also be saved to a text file. 

As a method of evaluating the quality of the alignment, if the needed path is set all points used for alignment will be projected into a single pointcloud and saved as a ply. An example of such a pointcloud can be seen below.

![example_pointcloud](https://user-images.githubusercontent.com/730680/48580820-7969c400-e920-11e8-9a69-e8aab2e74d20.png)

## CSV format

| Column | Description |
|--:|:--|
1 | timestamp ns 
2 | vertex index (not used) 
3 | position x
4 | position y
5 | position z 
6 | orientation quaternion w 
7 | orientation quaternion x
8 | orientation quaternion y 
9 | orientation quaternion z 

Note that Maplab has two CSV exporters. This file-format is the same as produced by [exportPosesVelocitiesAndBiasesToCsv](https://github.com/ethz-asl/maplab/blob/master/console-plugins/vi-map-data-import-export-plugin/src/export-vertex-data.cc#L39) but differs from the output of [exportVerticesAndTracksToCsv](https://github.com/ethz-asl/maplab/blob/master/tools/csv-export/src/csv-export.cc#L35)

## Parameters
------

### Scan Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `min_point_distance` |  Minimum range a point can be from the lidar and still be included in the optimization. | 0.0 |
| `max_point_distance` |  Maximum range a point can be from the lidar and still be included in the optimization. | 100.0 |
| `keep_points_ratio` |  Ratio of points to use in the optimization (runtimes increase drastically as this is increased). | 0.01 |
| `min_return_intensity` | The minimum return intensity a point requires to be considered valid. | -1.0 |
| `motion_compensation` |  If the movement of the lidar during a scan should be compensated for. | true |
| `estimate_point_times` | Uses the angle of the points in combination with `lidar_rpm` and `clockwise_lidar` to estimate the time a point was taken at. | false |
| `lidar_rpm` | Spin rate of the lidar in rpm, only used with `estimate_point_times`. | 600 |
| `clockwise_lidar` | True if the lidar spins clockwise, false for anti-clockwise, only used with `estimate_point_times`. | false |

### IO Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `use_n_scans` |  Optimization will only be run on the first n scans of the dataset. | 2147483647 |
| `input_bag_path` |  Path of rosbag containing sensor_msgs::PointCloud2 messages from the lidar. | N/A  |
| `transforms_from_csv` | True to load scans from a csv file, false to load from the rosbag. | false |
| `input_csv_path` |  Path of csv generated by Maplab, giving poses of the system to calibrate to. | N/A |
| `output_pointcloud_path` |  If set, a fused pointcloud will be saved to this path as a ply when the calibration finishes. | "" |
| `output_calibration_path` |  If set, a text document giving the final transform will be saved to this path when the calibration finishes. | "" |

### Alinger Parameters
| Parameter | Description | Default |
| --------------------  |:-----------:| :-------:|
| `local` |  If False a global optimization will be performed and the result of this will be used in place of the `inital_guess` parameter. | false |
| `inital_guess` |  Initial guess to the calibration (x, y, z, rotation vector, time offset), only used if running in `local` mode. | [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] |
| `max_time_offset` |  Maximum time offset between sensor clocks in seconds. | 0.1 |
| `angular_range` | Search range in radians around the `inital_guess` during the local optimization stage. | 0.5 |
| `translational_range` | Search range around the `inital_guess` during the local optimization stage. | 1.0 |
| `max_evals` | Maximum number of function evaluations to run | 200 |
| `xtol` | Tolerance of final solution | 0.0001 |
| `knn_batch_size` | Number of points to send to each thread when finding nearest points | 1000 |
| `knn_k` | Number of neighbors to consider in error function | 1 |
| `global_knn_max_dist` | Error between points is limited to this value during global optimization. | 1.0 |
| `local_knn_max_dist` | Error between points is limited to this value during local optimization. | 0.1 |
| `time_cal` | True to perform time offset calibration | true |
