#include <sbmpo_ros/sbmpo_util.hpp>

namespace sbmpo {

    /*
        Heuristics Functions
    */

    void initializeBuffer(NodeBuffer &buffer, const size_t size) {
        buffer = new Node[size];
        for (Index idx = 0; idx < size; idx++)
            buffer[idx].id = -1;
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

    void initializeQueue(NodeQueue &queue, NodeBuffer &buffer) {
        const std::function<bool(int,int)> comp = [&](int a, int b) {
            return buffer[a].heuristic[0] > buffer[b].heuristic[0];
        };
        queue = NodeQueue(comp);
    }


    Node startingNode(const PlannerOptions &options) {
        Node starting_node;
        for (int s = 0; s < options.state_info.size(); s++)
            starting_node.state.push_back(options.state_info[s].initial_value);
        for (int c = 0; c < options.control_info.size(); c++)
            starting_node.control.push_back(options.control_info[c].initial_value);
        starting_node.id = 0;
        starting_node.parent_id = -1;
        starting_node.generation = 0;
        starting_node.heuristic = {h(starting_node), 0.0};
        return starting_node;
    }

    bool isGoal(const State &state, const StateInfoList &info_list) {
        for (int i = 0; i < state.size(); i++) {
            const StateInfo &info = info_list[i];
            const float val = state[i];
            if (info.defined_goal)
                if (val < info.goal_value[0] || val > info.goal_value[1])
                    return false;
        }
        return true;
    }

    float dg(const Node &node1, const Node &node2) {
        // TODO
        return 0.0;
    }

    float h(const Node &node) {
        // TODO
        return 0.0;
    }


    /*
        Sample Generation Functions
    */

    Control generateHaltonSamples(const int n, const unsigned int seed) {
        Control samples;
        double x;
        int k, p, num;
        for (int j = 0; j < n; j++) {
            x = 0.0, k = 1, p = primes[j], num = seed;
            while (num > 0) {
                x += double(num % p) / pow(p, k++);
                num /= p;
            }
            samples.push_back(x);
        }
        return samples;
    }

    Control generateRandomSamples(const int n, const unsigned int seed) {
        if (!seeded_rand) {
            srand(seed);
            seeded_rand = true;
        }
        Control samples;
        for (int j = 0; j < n; j++)
            samples.push_back(double(rand()) / RAND_MAX);
        return samples;
    }


    /*
        Implicit Grid Functions
    */

    void toGridKey(GridKey &key, const State &state, const ImplicitGrid &grid) {
        key.clear();
        const GridResolution &resolution = grid.resolution;
        for (int i = 0; i < state.size(); i++)
            if (grid.active[i])
                key.push_back(int(state[i]/resolution[i]));
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

    Index toNodeIndex(const State &state, const ImplicitGrid &grid) {
        GridKey key;
        toGridKey(key, state, grid);
        return grid.buffer[toGridIndex(key, grid)];
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

    void initialize(Planner &planner, const PlannerOptions &options) {
        planner.options = options;
        planner.buffer_size = options.max_iterations*options.sample_size + 1;
        initializeBuffer(planner.buffer, planner.buffer_size);
        initializeGrid(planner.grid, options);
        initializeQueue(planner.queue, planner.buffer);
        planner.results.best = 0;
        planner.results.high = 0;
    }

    void reset(Planner &planner) {
        for (Index idx = 0; idx < planner.results.high; idx++)
            planner.buffer[idx].id = -1;
        for (Index idx = 0; idx < planner.grid.max_size; idx++)
            planner.grid.buffer[idx] = -1;
        planner.queue = {};
        planner.results.best = 0;
        planner.results.high = 0;
        planner.results.path.clear();
    }

    void deconstruct(Planner &planner) {
        delete[] planner.buffer;
        delete[] planner.grid.buffer;
    }

}