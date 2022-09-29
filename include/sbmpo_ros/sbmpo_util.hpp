#ifndef SBMPO_UTIL_HPP
#define SBMPO_UTIL_HPP

#define HALTON_MAX_DIMENSIONS 10

#include <sbmpo_ros/sbmpo_types.hpp>
#include <math.h>
#include <rand.h>
#include <time.h>

namespace sbmpo {

    /*
        Heuristics Functions
    */

    // Determine if state is goal
    bool isGoal(const State &state, const State &goal);

    // G score of a given state
    float dg(const State &state1, const State &state2);

    // H score of a given state
    float h(const State &state);


    /*
        Sample Generation Functions
    */

    // Stores primes used in Halton sampling
    const int primes[HALTON_MAX_DIMENSIONS] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29}; 

    // Generate Halton samples
    Control generateHaltonSamples(const unsigned int i, const int n);

    // Generate random samples
    Control generateRandomSamples(const int n, const unsigned int seed = time(NULL));

    // Determine if rand() has been seeded
    static bool seeded_rand = false;


    /*
        Implicit Grid Functions
    */

    // Convert state position to implicit grid key
    void toGridKey(const State &state, const GridResolution &resolution, GridKey &key);

    // Convert implicit grid key to buffer index
    int toGridIndex(const GridKey &key, const GridSize &grid_size);

    // Convert state position directly to node buffer index
    int toNodeIndex(const State &state, const ImplicitGrid &grid);

    // Get total grid size
    int getTotalGridSize(const GridSize &grid_size);


    /*
        Planner Control Functions
    */

    // Initialize the planner with given options
    void initialize(Planner &planner, const PlannerOptions &options);

    // Reset planner
    void reset(Planner &planner);

    // Generate the starting
    Node generateStartingNode(const PlannerOptions &options);

}

#endif