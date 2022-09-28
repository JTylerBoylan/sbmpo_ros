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
        Utility for storing vertex information
    */

    struct Node {
        State state;
        Control control;
        int id;
        int parent_id;
        int generation;
    };


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

    /*
        Utility for configuring planner
    */

    struct StateInfo {
        std::string name;
        double range[2];
        bool grid;
        float grid_resolution;
        int grid_size;
    };

    void configure(std::vector<StateInfo> &states);

}

#endif