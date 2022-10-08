#ifndef SBMPO_EXTERN_HPP
#define SBMPO_EXTERN_HPP

#include <sbmpo_ros/sbmpo_util.hpp>
#include <ros/console.h>
#include <ros/package.h>

namespace sbmpo_ext {

    // Initialize the planning
    extern bool initialize(sbmpo::Planner &planner);

    // Evaluate a node with a control
    extern bool evaluate(sbmpo::Node &node, const sbmpo::Planner &planner, const int n);

    // Send external data into sbmpo
    template<class T> 
    extern void send_external(T &obj);

    // Get external data from sbmpo
    template<class T>
    extern void get_external(T &obj);

}

#endif