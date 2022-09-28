# Hybrid A* Local Planner

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
git clone https://github.com/JTylerBoylan/hybrid_astar
```

## Building & Running
### In `catkin_ws`
```
catkin_make
source devel/setup.bash
roslaunch hybrid_astar planner.launch
```

## Parameters
Some parameters can be set from the `planner.launch` file. This is how you change the map file, the starting odometry, the goal point, and the topics & frame ids of the planner. Changing these parameters doesn't require you to rebuild the package.

The planner parameters are set in the `planner.cpp` file in the `src` folder. This is how you change sampling velocities, sampling time, cost factors, etc.
You will have to rebuild the package using `catkin_make` after changing these parameters.
