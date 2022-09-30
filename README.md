# SBMPO Planner for ROS

## Dependencies
### ROS Noetic
### Grid Map
```
sudo apt-get install ros-noetic-grid-map
```
### Eigen
```
sudo apt-get install libeigen3-dev
```

## Installation
### In `catkin_ws/src`
```
git clone https://github.com/JTylerBoylan/sbmpo_ros
```

## Building & Running
### In `catkin_ws`
```
catkin_make
source devel/setup.bash
roslaunch sbmpo_ros sbmpo.launch
```

## Parameters
Parameters located in `src/sbmpo_extern.cpp`, `config/states.yaml`, and `config/config.yaml`

## Notes
Halton and Random sampling don't work yet.
Also, it hasn't been tested on anything but the thermal grid on one set of parameters, so there still may be bugs
