#ifndef SBMPO_UTIL_HPP
#define SBMPO_UTIL_HPP

#define HALTON_MAX_DIMENSIONS 10

#include <sbmpo_ros/sbmpo_extern.hpp>
#include <math.h>
#include <stdlib.h>
#include <time.h>

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

    // Determine if state is goal
    bool isGoal(const State &state, const StateInfoList &info_list);

    // Subtract g score from all successor nodes by some value
    void updateSuccessors(Node &node, Planner &planner, const float diff, const Index start);


    /*
        Sample Generation Functions
    */

    // Stores primes used in Halton sampling
    const int primes[HALTON_MAX_DIMENSIONS] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29}; 

    // Generate Halton samples
    Control generateHaltonSamples(const int n, const unsigned int seed = 123);

    // Generate random samples
    Control generateRandomSamples(const int n, const unsigned int seed = time(NULL));

    // Determine if rand() has been seeded
    static bool seeded_rand = false;

    // General form of generate samples
    Control generateSamples(const PlannerOptions &options, const int index, const int n);

    /*
        Implicit Grid Functions
    */

    // Convert state position to implicit grid key
    GridKey toGridKey(const State &state, const ImplicitGrid &grid);

    // Convert implicit grid key to buffer index
    Index toGridIndex(const GridKey &key, const ImplicitGrid &grid);

    // Convert node to grid index directly
    Index& toNodeIndex(const Node &node, const ImplicitGrid &grid);

    // Get total grid size
    size_t totalGridSize(const GridSize &grid_size);


    /*
        Planner Control Functions
    */

    // Initialize the planner with given options
    void initialize(Planner &planner);

    // Reset planner
    void reset(Planner &planner);

    // Delete buffers
    void deconstruct(Planner &planner);

    /*
        Conversions
    */

    // Convert string to sample type
    SampleType toSampleType(const std::string &type);

}

#endif