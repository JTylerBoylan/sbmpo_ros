#ifndef SBMPO_EXTERN_HPP
#define SBMPO_EXTERN_HPP

#include <sbmpo_ros/sbmpo_types.hpp>

namespace sbmpo {

    // Evaluate a node with a control
    bool evaluate(Node &node, const Control &control, const Planner &planner);

    // Send external data into sbmpo
    template<class T> void send_external(T &obj);

}

#endif