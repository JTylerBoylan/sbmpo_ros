#include <sbmpo_ros/sbmpo_util.hpp>

namespace sbmpo {

    std::vector<double> generateHaltonSamples(const unsigned int i, const int n) {
        std::vector<double> samples;
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

    std::vector<double> generateRandomSamples(const int n, const unsigned int seed = time(NULL)) {
        srand(seed);
        std::vector<double> samples;
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

    int toGridIndex(const State &state, const GridResolution &resolution, const GridSize &grid_size) {
        GridKey key;
        toGridKey(state, resolution, key);
        return toGridIndex(key, grid_size);
    }

}