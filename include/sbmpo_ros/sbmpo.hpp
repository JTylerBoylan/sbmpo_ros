#ifndef SBMPO_HPP
#define SBMPO_HPP

#include <sbmpo_ros/sbmpo_util.hpp>
#include <grid_map_core/GridMap.hpp>

namespace sbmpo {

    void run(Planner &planner);

    int sample(const Node &node, const int n);

}

#endif