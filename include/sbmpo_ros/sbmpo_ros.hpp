#ifndef SBMPO_ROS_HPP
#define SBMPO_ROS_HPP

#include <ros/ros.h>
#include <sbmpo_ros/sbmpo.hpp>
#include <map>

namespace sbmpo {

    // Configure a planner using a node handle's parameters
    void configure(Planner &planner, const ros::NodeHandle handle);

    // Generate samples
    void generateSampleList(SampleList &sample_list, const XmlRpc::XmlRpcValue &samples);

    // Parse state name
    void parseStateName(const XmlRpc::XmlRpcValue &value, StateInfo &info);

    // Parse state value section
    void parseStateValue(const XmlRpc::XmlRpcValue &value, StateInfo &info);

    // Parse state grid section
    void parseStateGrid(const XmlRpc::XmlRpcValue &value, ImplicitGrid &grid);

    // Parse state control section
    void parseStateControl(const XmlRpc::XmlRpcValue &value, StateInfo &info);

    // Parse control name
    void parseControlName(const XmlRpc::XmlRpcValue &value, ControlInfo &info);

    // Parse control value section
    void parseControlValue(const XmlRpc::XmlRpcValue &value, ControlInfo &info);

}

#endif