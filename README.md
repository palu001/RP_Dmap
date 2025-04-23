# Distance Map Localizer - ROS Node

## Overview
This ROS node implements a distance map-based localization system that:
- Processes occupancy grid maps to create distance maps
- Uses scan matching (ICP) to localize the robot
- Publishes corrected odometry

## Dependencies
- ROS (tested with Noetic)
- Eigen3
- `rp_stuff` package (contains DMap implementation)

## Topics

### Subscribed Topics
- `/map` (`nav_msgs/OccupancyGrid`): Input map
- `/initialpose` (`geometry_msgs/PoseWithCovarianceStamped`): Initial pose from Rviz
- `/base_scan` (`sensor_msgs/LaserScan`): Laser scans

### Published Topics
- `/odom_corrected` (`nav_msgs/Odometry`): Corrected odometry

## Launching
In every terminal run
```bash
source /opt/ros/noetic/setup.bash
```
and in the one for the `listener node` use
```bash
source devel/setup.bash
```
Build the project after cloning the repository:
```bash
catkin build
```
Then on different terminals:

```bash
roscore
```

```bash
rosrun listener listener_node
```

```bash
rosrun map_server map_server <path/to/map.yaml>
```

```bash
rviz
```

```bash
rosrun stage_ros stageros <path/to/map.world>
```