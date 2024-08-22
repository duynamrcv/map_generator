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
sudo apt install ros-noetic-pcl
```
### Setup and build
```
cd <workspace>/src
git clone git@github.com:duynamrcv/map_generator.git
cd <workspace>
catkin_make
```

## Demo
```
cd <workspace>
source devel/setup.bash
roslaunch map_generator single_run_in_sim.launch
```
<img src="data/image.png" alt="" width="100%"/>