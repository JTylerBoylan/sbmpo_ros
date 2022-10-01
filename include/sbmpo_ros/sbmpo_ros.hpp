#ifndef SBMPO_ROS_HPP
#define SBMPO_ROS_HPP

#include <ros/ros.h>
#include <sbmpo_ros/sbmpo.hpp>
#include <grid_map_core/GridMap.hpp>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <map>

namespace sbmpo {

    // Configure a planner using a node handle's parameters
    void configure(Planner &planner, const ros::NodeHandle handle);

    // Get nav_msgs::Path from planner
    void getPath(nav_msgs::Path& path, const Planner &planner, const grid_map::GridMap &map);

    // Get geometry_msgs::PoseArray
    void getAllPoses(geometry_msgs::PoseArray &poses, const Planner &planner, const grid_map::GridMap &map);

    // Convert Node to geometry_msgs::Pose
    geometry_msgs::Pose toPose(const Node &node, const grid_map::GridMap &map);

}

#endif