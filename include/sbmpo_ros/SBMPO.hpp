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

            // List to store state information
            StateInfoList states;

            // List to store control information
            ControlInfoList controls;

            // Maximum iterations of planner
            int max_iterations;

            // Maximum generations of planner
            int max_generations;

            // Sample size
            int sample_size;

            // Implicit grid size
            GridSize implicit_grid_size;

            // Implicit grid resolution
            GridResolution implicit_grid_resolution;

            // Goal state
            State goal;

            // Best generated path
            std::vector<int> path;

    };

}


#endif