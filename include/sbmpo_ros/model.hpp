#ifndef MODEL_HPP
#define MODEL_HPP

#include <sbmpo_ros/sbmpo/sbmpo_util.hpp>
#include <ros/console.h>
#include <ros/package.h>

namespace model {

    // Initialize the planning
    extern bool initialize(sbmpo::Planner &planner);

    // Evaluate a node with a control
    extern bool next_state(sbmpo::Node &node, const int n);

    // Get the cost of a control
    extern float cost(const sbmpo::Node& current, const sbmpo::Node &next);

    // Get the heuristic of a node
    extern float heuristic(const sbmpo::Node& current, const sbmpo::State &goal);

    // Determine if state is valid
    extern bool is_valid(const sbmpo::State& state);

    // Determine if a state is the goal
    extern bool is_goal(const sbmpo::State& state);

    // Send external data into sbmpo
    template<class T> 
    extern void send_external(T &obj);

    // Get external data from sbmpo
    template<class T>
    extern void get_external(T &obj);

}

#endif