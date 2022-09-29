#ifndef SBMPO_ROS_HPP
#define SBMPO_ROS_HPP

#include <ros/ros.h>
#include <sbmpo_ros/sbmpo.hpp>

namespace sbmpo {

    void configure(Planner &planner, const ros::NodeHandle handle);

}

#endif