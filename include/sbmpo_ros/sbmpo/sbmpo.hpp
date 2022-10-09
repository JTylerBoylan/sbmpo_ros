#ifndef SBMPO_HPP
#define SBMPO_HPP

#include <sbmpo_ros/model.hpp>
#include <sbmpo_ros/sbmpo/sbmpo_util.hpp>

namespace sbmpo {;

    // Run the planner
    void run(Planner &planner);

    // Sample a specific node
    Index sample(Planner &planner, const Node &node, const int n);

}

#endif