#ifndef SBMPO_TYPES_HPP
#define SBMPO_TYPES_HPP

#include <vector>
#include <queue>
#include <array>

namespace sbmpo {

    // State: n-dimensional array of state positions
    typedef std::vector<float> State;

    // Control: n-dimensional array of controls
    typedef std::vector<float> Control;

    // Heuristic: Stores the g & f scores
    typedef std::array<float, 2> Heuristic;


    // Struct to hold node information
    struct Node {
        State state;
        Control control;
        Heuristic heuristic;
        int id;
        int parent_id;
        int generation;
    };

    // Array of nodes
    typedef Node* NodeBuffer;

    // Types for implicit grid
    typedef std::vector<int> GridKey;
    typedef std::vector<float> GridResolution;
    typedef std::vector<int> GridSize;

    // Struct to hold implicit grid information
    struct ImplicitGrid {
        GridSize size;
        GridResolution resolution;
        int max_size;
        int * buffer;
    };

    // Struct to hold state information
    struct StateInfo {
        std::string name;
        double range[2];
        float initial_value;
        float goal_value[2];
        bool grid;
        float grid_resolution;
        int grid_size;
    };

    // Struct to hold control information
    struct ControlInfo {
        std::string name;
        float initial_value;
        double range[2];
    };

    // Types for lists of states and controls
    typedef std::vector<StateInfo> StateInfoList;
    typedef std::vector<ControlInfo> ControlInfoList;

    // Struct for planner options
    struct PlannerOptions {
        int max_iterations;
        int max_generations;
        int sample_size;
        int max_size;
        StateInfoList state_info;
        ControlInfoList control_info;
        State goal;
    };

    // Type to hold planner path
    typedef std::vector<int> Path;

    struct PlannerResults {
        int high;
        int best;
        Path path;
    };

    // Type for priority queue
    typedef std::priority_queue<int, std::vector<int>, const std::function<bool(int,int)>>* NodeQueue;

    // Struct to hold all planner information
    struct Planner {
        PlannerOptions options;
        NodeBuffer buffer;
        ImplicitGrid grid;
        NodeQueue queue;
        PlannerResults results;
    };

}

#endif