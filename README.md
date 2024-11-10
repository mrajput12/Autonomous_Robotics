# Autonomous Drone for 3D Mapping and Scanning
## Introduction
---
This repository is being developed  for Simulating Warehouse Automation using drone.
- Drone with Warehouse Simulation is done in Gazebo.
- Hove/ Fly the drone with custom node.
- Creating a RGB 3D Point Cloud Map using RTAB-Map Package.
- Processing pointcloud map with custom node to segments rack's legs.
- Text file is generated with mid points of racks for robot to navigate.

## Future Work and Ideas
---
- Setting Mid point extraction to parameters
- Rtab_mapviz takes location saving and utilize older map for completing automating segmentation pipeline
- Fix move base planners
    - avoid obstacles with more space in between
- Shifting from Rtabmap to Octomap for faster computation
## Features
---
### 1. Navigating to Specific Rack
Using hard coded locations of racks extracted through rtab_map point clouds
- Bringing the Simulation
```
roslaunch uav_sim  3_move_base_goal_mapping.launch
```
- Sending Rack goals with Rack Names
```
rosrun uav_sim single_coordinate_navigation.py c3 # c row 3 column
```
### 2. Perform Mapping with RtabMap
- We are performing RtabMapping for point cloud segmentation , process to launch its files is
```
roslaunch uav_sim 2_build_3D_map_rtabmap.launch
```
- After scanning whole warehouse, pause the RtabVIZ and export as pcd with increasing voxeliziing size to reduce map size

## Files in this Package
---
### Launch Files
- Bring drone within Warehouse
```
roslaunch uav_sim 1_drone_warehouse_sim.launch
```
- Build 3D rtabmap
```
roslaunch uav_sim 2_build_3D_map_rtabmap.launch
```
- Move Base integeration for goal based motion
```
roslaunch uav_sim 3_move_base_goal_mapping.launch
```
### Nodes
- Make the drone Hover
```
rosrun uav_sim hover_drone.py
```
- Drive Drone using teleoperation keyboard
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
- Drone Motion from left to right and repeat for scanning
```
rosrun uav_sim rack_scanning_sub_auto.py
```
- Extracting Mid points between racks from 3D point cloud
```
rosrun uav_sim rtabmap_rack_mid_points.py
```
- Move drone to a specified rack name
```
rosrun uav_sim single_coordinate_navigation <rack_name>
```

## Software Requirments
---
- Ubuntu 20.04
- ROS Noetic

- Install package dependencies
```
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-geographic-msgs
sudo apt-get install ros-noetic-map-server
sudo apt-get install ros-noetic-amcl
sudo apt-get install ros-noetic-slam-gmapping
sudo apt-get install ros-noetic-slam-toolbox
sudo apt-get install ros-noetic-rtabmap
sudo apt-get install ros-noetic-rtabmap-ros
```
## Installing this Package
- This repository is a package , clone it inside of your ros workspace
```
git clone <repolink> ## inside ~/catkin_ws/src
```
- Build repository through
```
catkin_make ## inside of catkin_ws
```
- Source workspace
```
source ~/catkin_ws/src/devel/setup.bash file
```

