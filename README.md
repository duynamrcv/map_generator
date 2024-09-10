# Point Cloud Map Generation
This repository provides the point cloud map generation for various planing methods based on ROS.

## Installation
### Prerequisites
Install clang-format
```
sudo apt install clang-format-10
sudo ln -s /usr/bin/clang-format-10 /usr/local/bin/clang-format
```
Install needed library
```
sudo apt install ros-humble-pcl-ros
```
### Setup and build
```
cd <workspace>/src
git clone git@github.com:duynamrcv/random_map.git
cd <workspace>
catkin_make
```

## Demo
```
cd <workspace>
source install/local_setup.bash
ros2 launch random_map random_forest.launch.py
```
<img src="doc/sample.png" alt="" width="100%"/>