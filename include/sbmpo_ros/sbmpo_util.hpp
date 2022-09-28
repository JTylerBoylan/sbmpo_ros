#ifndef SBMPO_UTIL_HPP
#define SBMPO_UTIL_HPP

#define HALTON_MAX_DIMENSIONS 10

#include <math.h>
#include <vector>
#include <array>
#include <rand.h>
#include <time.h>

namespace sbmpo {

    // State: n-dimensional array of state positions
    typedef std::vector<float> State;

    // Control: Stores the g & f scores
    typedef std::array<float, 2> Control;


    /*
        Utility for storing node information
    */

    struct Node {
        State state;
        Control control;
        int id;
        int parent_id;
        int generation;
    };

    /*
        Utility for heuristics
    */

    // G score of a given state
    float dg(const State &state1, const State &state2);

    // H score of a given state
    float h(const State &state);


    /*
        Utility for generating samples
    */

    // Stores primes used in Halton sampling
    const int primes[HALTON_MAX_DIMENSIONS] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29}; 

    // Generate Halton samples
    std::vector<double> generateHaltonSamples(const unsigned int i, const int n);

    // Generate random samples
    std::vector<double> generateRandomSamples(const int n, const unsigned int seed = time(NULL));


    /*
        Utility for implicit gridding
    */

   typedef std::vector<int> GridKey;
   typedef std::vector<float> GridResolution;
   typedef std::vector<int> GridSize;

    // Convert state position to implicit grid key
    void toGridKey(const State &state, const GridResolution &resolution, GridKey &key);

    // Convert implicit grid key to buffer index
    int toGridIndex(const GridKey &key, const GridSize &grid_size);

    // Convert state position directly to buffer index
    int toGridIndex(const State &state, const GridResolution &resolution, const GridSize &grid_size);

    // Get total grid size
    int getTotalGridSize(const GridSize &grid_size);

    /*
        Utility for configuring planner
    */

    struct StateInfo {
        std::string name;
        double range[2];
        float initial_value;
        float goal_value;
        bool grid;
        float grid_resolution;
        int grid_size;
    };

    typedef std::vector<StateInfo> StateInfoList;

    void configure(StateInfoList &states, GridSize &grid_size, GridResolution &grid_resolution);

    Node generateStartingNode(const StateInfoList &states);

}

#endif