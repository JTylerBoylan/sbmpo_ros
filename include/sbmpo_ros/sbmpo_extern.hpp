#ifndef SBMPO_EXTERN_HPP
#define SBMPO_EXTERN_HPP

#include <sbmpo_ros/sbmpo_types.hpp>

namespace sbmpo {

    // Evaluate a node with a control
    extern bool evaluate(Node &node, const Planner &planner, const int n);

    // Send external data into sbmpo
    template<class T> 
    extern void send_external(T &obj);

    // Get external data from sbmpo
    template<class T>
    extern void get_external(T &obj);

}

#endif