#ifndef SBMPO_EXTERN_HPP
#define SBMPO_EXTERN_HPP

#include <sbmpo_ros/sbmpo_types.hpp>

namespace sbmpo {

    // Evaluate a node with a control
    void evaluate(Node &node, const Control &control, const Planner &planner);

    // G score of a given state
    void calculateG(Node &node, const Node &parent, const Planner &planner);

    // H score of a given state
    void calculateH(Node &node, const Planner &planner);

}

#endif