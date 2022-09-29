#ifndef SBMPO_ROS_HPP
#define SBMPO_ROS_HPP

#include <ros/ros.h>
#include <sbmpo_ros/sbmpo.hpp>
#include <map>

namespace sbmpo {

    // Configure a planner using a node handle's parameters
    void configure(Planner &planner, const ros::NodeHandle handle);

}

#endif