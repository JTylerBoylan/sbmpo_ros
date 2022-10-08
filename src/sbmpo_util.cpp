#include <sbmpo_ros/sbmpo_util.hpp>

namespace sbmpo {

    /*
        Heuristics Functions
    */

    void initializeBuffer(NodeBuffer &buffer, const size_t size) {
        buffer = new Node[size];
    }

    void initializeGrid(ImplicitGrid &grid, const PlannerOptions &options) {
        GridActive &active = grid.active;
        GridSize &size = grid.size;
        GridResolution &resolution = grid.resolution;
        for (int i = 0; i < options.state_info.size(); i++) {
            const StateInfo &info = options.state_info[i];
            if (info.grid) {
                active.push_back(true);
                size.push_back(info.grid_size);
                resolution.push_back(info.grid_resolution);
            } else {
                active.push_back(false);
            }
        }
        grid.max_size = totalGridSize(size);
        grid.buffer = new Index[grid.max_size];
    }

    Node startingNode(const PlannerOptions &options) {
        Node starting_node;
        for (int s = 0; s < options.state_info.size(); s++)
            starting_node.state.push_back(options.state_info[s].initial_value);
        for (int c = 0; c < options.control_info.size(); c++)
            starting_node.control.push_back(options.control_info[c].initial_value);
        starting_node.id = 0;
        starting_node.parent_id = INVALID_INDEX;
        starting_node.generation = 0;
        starting_node.heuristic = {INFINITY, 0.0};
        return starting_node;
    }

    bool isGoal(const State &state, const StateInfoList &info_list) {
        float sum = 0.0f;
        for (int i = 0; i < state.size(); i++) {
            const StateInfo &info = info_list[i];
            const float val = state[i];
            if (info.goal_radius != -1.0f)
                sum += powf((state[i] - info.goal_value) / info.goal_radius, 2.0f);
        }
        return sum <= 1.0f;
    }

    void updateSuccessors(Node &node, Planner& planner, const float diff, const Index start) {
        Index child_start = node.child_id;
        if (child_start != INVALID_INDEX && child_start != start)
            for (int c = 0; c < planner.options.sample_size; c++) {
                Node& child = planner.buffer[child_start + c];
                child.heuristic[1] -= diff;
                updateSuccessors(child, planner, diff, start);
            }
    }


    /*
        Implicit Grid Functions
    */

    GridKey toGridKey(const State &state, const ImplicitGrid &grid) {
        GridKey key;
        const GridResolution &resolution = grid.resolution;
        for (int i = 0; i < state.size(); i++)
            if (grid.active[i])
                key.push_back(int(state[i]/resolution[i]));
        return key;
    }

    Index toGridIndex(const GridKey &key, const ImplicitGrid &grid) {
        Index index = 0;
        for (int i = 0; i < key.size(); i++) {
            int step = 1;
            for (int j = 0; j < i; j++)
                step *= grid.size[j];
            index += key[i]*step;
        }
        return index;
    }

    Index& toNodeIndex(const Node &node, const ImplicitGrid &grid) {
        const GridKey key = toGridKey(node.state, grid);
        const Index index = toGridIndex(key, grid);
        return grid.buffer[index];
    }

    size_t totalGridSize(const GridSize &grid_size) {
        size_t size = 1;
        for (int i = 0; i < grid_size.size(); i++)
            size *= grid_size[i];
        return size;
    }

    /*
        Planner Control Functions
    */

    void initializePlanner(Planner &planner) {
        planner.buffer_size = planner.options.max_iterations*planner.options.sample_size + 1;
        initializeBuffer(planner.buffer, planner.buffer_size);
        initializeGrid(planner.grid, planner.options);
        planner.results.high = planner.buffer_size;
    }

    void resetPlanner(Planner &planner) {
        for (Index idx = 0; idx < planner.results.high; idx++) {
            planner.buffer[idx].id = -1;
            planner.buffer[idx].child_id = -1;
        }
        for (Index idx = 0; idx < planner.grid.max_size; idx++)
            planner.grid.buffer[idx] = -1;
        planner.results.best = 0;
        planner.results.high = 0;
        planner.results.path.clear();
        planner.buffer[0] = startingNode(planner.options);
        toNodeIndex(planner.buffer[0], planner.grid) = 0;
    }

    void deconstructPlanner(Planner &planner) {
        delete[] planner.buffer;
        delete[] planner.grid.buffer;
    }

}