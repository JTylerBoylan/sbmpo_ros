cmake_minimum_required(VERSION 3.0.2)
project(sbmpo_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  grid_map_ros
  roscpp
  nav_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sbmpo_ros
  CATKIN_DEPENDS grid_map_ros roscpp nav_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/sbmpo_util.cpp
  src/sbmpo.cpp
  src/sbmpo_ros.cpp
  src/sbmpo_extern.cpp
)

add_executable(sbmpo src/ros_node.cpp)
target_link_libraries(sbmpo ${catkin_LIBRARIES} ${PROJECT_NAME})

catkin_install_python(PROGRAMS scripts/mat2gridmap.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

