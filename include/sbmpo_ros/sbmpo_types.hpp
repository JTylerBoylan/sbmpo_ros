#ifndef SBMPO_TYPES_HPP
#define SBMPO_TYPES_HPP

#include <vector>
#include <queue>
#include <array>
#include <functional>
#include <algorithm>

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
        State state;
        Control control;
        Heuristic heuristic;
        Index id;
        Index parent_id;
        int generation;
    };

    // Array of nodes
    typedef Node* NodeBuffer;

    // Types for implicit grid
    typedef std::vector<bool> GridActive;
    typedef std::vector<int> GridKey;
    typedef std::vector<float> GridResolution;
    typedef std::vector<int> GridSize;

    // Struct to hold implicit grid information
    struct ImplicitGrid {
        GridActive active;
        GridSize size;
        GridResolution resolution;
        int max_size;
        Index * buffer;
    };

    // Type to indicate array is a range
    typedef std::array<float, 2> Range;

    // Type to hold pointer to control equation
    typedef float (*ControlEquation)(float*, float*);

    // Struct to hold state information
    struct StateInfo {
        std::string name;
        Range range;
        float initial_value;
        Range goal_value;
        bool defined_goal;
        bool grid;
        float grid_resolution;
        int grid_size;
        ControlEquation equation;
    };

    // Struct to hold control information
    struct ControlInfo {
        std::string name;
        float initial_value;
        Range range;
    };

    // Types for sample storage
    typedef std::vector<std::vector<float>> SampleList;

    // Types for lists of states and controls
    typedef std::vector<StateInfo> StateInfoList;
    typedef std::vector<ControlInfo> ControlInfoList;

    // Struct for planner options
    struct PlannerOptions {
        int max_iterations;
        int max_generations;
        int sample_size;
        std::string sample_type;
        SampleList sample_list; // Only used if sample type is 'input'
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
    typedef std::priority_queue<int, std::vector<Index>, std::function<bool (int,int)>> NodeQueue;

    // Struct to hold all planner information
    struct Planner {
        PlannerOptions options;
        NodeBuffer buffer;
        ImplicitGrid grid;
        NodeQueue queue;
        PlannerResults results;
        size_t buffer_size;
    };

}

#endif