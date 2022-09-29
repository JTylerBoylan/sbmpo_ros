#include <sbmpo_ros/sbmpo_util.hpp>

namespace sbmpo {

    Control generateHaltonSamples(const unsigned int i, const int n) {
        Control samples;
        double x;
        int k, p, num;
        for (int j = 0; j < n; j++) {
            x = 0.0, k = 1, p = primes[j], num = i;
            while (num > 0) {
                x += double(num % p) / pow(p, k++);
                num /= p;
            }
            samples.push_back(x);
        }
        return samples;
    }

    Control generateRandomSamples(const int n, const unsigned int seed = time(NULL)) {
        if (!seeded_rand) {
            srand(seed);
            seeded_rand = true;
        }
        Control samples;
        for (int j = 0; j < n; j++)
            samples.push_back(double(rand()) / RAND_MAX);
        return samples;
    }

    void toGridKey(const State &state, const GridResolution&resolution, GridKey &key) {
        key.clear();
        for (int i = 0; i < state.size(); i++)
            key.push_back(int(state[i]/resolution[i]));
    }

    int toGridIndex(const GridKey &key, const GridSize &grid_size) {
        int index = 0;
        for (int i = 0; i < key.size(); i++) {
            int step = 1;
            for (int j = 0; j < i; j++)
                step *= grid_size[j];
            index += key[i]*step;
        }
        return index;
    }

    int toGridIndex(const State &state, const ImplicitGrid &grid) {
        GridKey key;
        toGridKey(state, grid.resolution, key);
        return toGridIndex(key, grid.size);
    }

    int getTotalGridSize(const GridSize &grid_size) {
        int size = 1;
        for (int i = 0; i < grid_size.size(); i++)
            size *= grid_size[i];
        return size;
    }

    Node generateStartingNode(const PlannerOptions &options) {
        Node starting_node;
        for (int s = 0; s < options.state_info.size(); s++)
            starting_node.state.push_back(options.state_info[s].initial_value);
        for (int c = 0; c < options.control_info.size(); c++)
            starting_node.control.push_back(options.control_info[c].initial_value);
        starting_node.id = 0;
        starting_node.parent_id = -1;
        starting_node.generation = 0;
        starting_node.heuristic = {h(starting_node.state), 0.0};
        return starting_node;
    }

}