#ifndef SBMPO_TYPES_HPP
#define SBMPO_TYPES_HPP

#include <vector>
#include <queue>
#include <array>
#include <functional>
#include <algorithm>
#include <unordered_map>

#define INVALID_INDEX -1

namespace sbmpo {

    // State: n-dimensional array of state positions
    typedef std::vector<float> State;

    // Control: n-dimensional array of controls
    typedef std::vector<float> Control;

    // Heuristic: Stores the g & f scores
    typedef std::array<float, 2> Heuristic;

    // Index: Stores location in buffer
    typedef int Index;


    // Struct to hold node information
    struct Node {
        Index id;
        State state;
        Control control;
        Heuristic heuristic;
        Index parent_id;
        Index child_id;
        int generation;
    };

    // Array of nodes
    typedef Node* NodeBuffer;

    // Types for implicit grid
    typedef std::vector<bool> GridActive;
    typedef std::vector<int> GridKey;
    typedef std::vector<float> GridResolution;
    typedef std::vector<int> GridSize;
    typedef std::unordered_map<Index, Index> IndexKeyMap;

    // Struct to hold implicit grid information
    struct ImplicitGrid {
        GridActive active;
        GridSize size;
        GridResolution resolution;
        int max_size;
        IndexKeyMap map;
    };

    // Struct to hold state information
    struct StateInfo {
        std::string name;
        float initial_value;
        float goal_value;
        float goal_radius;
        bool grid;
        float grid_resolution;
        int grid_size;
    };

    // Struct to hold control information
    struct ControlInfo {
        std::string name;
        float initial_value;
        float range_min;
        float range_max;
    };

    // Types for lists of states and controls
    typedef std::vector<StateInfo> StateInfoList;
    typedef std::vector<ControlInfo> ControlInfoList;

    // Struct for planner options
    struct PlannerOptions {
        int max_iterations;
        int max_generations;
        int sample_size;
        float sample_time;
        float sample_time_increment;
        StateInfoList state_info;
        ControlInfoList control_info;
    };

    // Type to hold planner path
    typedef std::vector<Index> Path;

    struct PlannerResults {
        int high;
        int best;
        Path path;
    };

    // Type for priority queue
    typedef std::priority_queue<Index, std::vector<Index>, const std::function<bool (Index,Index)>> NodeQueue;

    // Struct to hold all planner information
    struct Planner {
        PlannerOptions options;
        NodeBuffer buffer;
        ImplicitGrid grid;
        PlannerResults results;
        size_t buffer_size;
    };

}

#endif