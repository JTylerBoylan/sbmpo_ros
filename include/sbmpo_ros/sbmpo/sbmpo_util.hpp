#ifndef SBMPO_UTIL_HPP
#define SBMPO_UTIL_HPP

#include <sbmpo_ros/sbmpo/sbmpo_types.hpp>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <fstream>
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream
#include <map>

namespace sbmpo {

    /*
        Initialization functions
    */

    // Initialize the node buffer with given options
    void initializeBuffer(NodeBuffer &buffer, const size_t size);

    // Initialize the implict grid with given options
    void initializeGrid(ImplicitGrid &grid, const PlannerOptions &options);

    /*
        Node Functions
    */

    // Generate the starting
    Node startingNode(const PlannerOptions &options);

    // Subtract g score from all successor nodes by some value
    void updateSuccessors(Node &node, Planner &planner, const float diff, const Index start);

    /*
        Implicit Grid Functions
    */

    // Convert state position to implicit grid key
    GridKey toGridKey(const State &state, const ImplicitGrid &grid);

    // Convert implicit grid key to buffer index
    Index toGridIndex(const GridKey &key, const ImplicitGrid &grid);

    // Convert node to grid index directly
    Index& toNodeIndex(const Node &node, ImplicitGrid &grid);

    // Get total grid size
    size_t totalGridSize(const GridSize &grid_size);


    /*
        Planner Control Functions
    */

    // Initialize the planner with given options
    void initializePlanner(Planner &planner);

    // Reset planner
    void resetPlanner(Planner &planner);

    // Delete buffers
    void deconstructPlanner(Planner &planner);

    /*
        Data File Reading
    */
   
    std::map<std::string, std::vector<float>> read_csv(std::string filename);

}

#endif