#ifndef SBMPO_HPP
#define SBMPO_HPP

#include <sbmpo_ros/model.hpp>
#include <sbmpo_ros/sbmpo/sbmpo_util.hpp>

namespace sbmpo {;

    void run(Planner &planner);

    Index sample(Planner &planner, const Node &node, const int n);

}

#endif