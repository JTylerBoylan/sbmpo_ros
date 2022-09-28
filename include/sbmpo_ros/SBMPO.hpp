#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <ros/ros.h>
#include <sbmpo_ros/sbmpo_util.hpp>
#include <grid_map_core/GridMap.hpp>

#include <queue>
#include <vector>

namespace sbmpo {

    class SBMPO {

        public:

            // Constructor
            SBMPO();

            // Destructor
            ~SBMPO();

            // Run the SBMPO algorithm
            void run();

        private:

            // Buffer array for all nodes
            Node * buffer;

            // Array to store state information
            std::vector<StateInfo> states;

            // Maximum iterations of planner
            int max_iterations;

            // Sample size
            int sample_size;
    };

}


#endif