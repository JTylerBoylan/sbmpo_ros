#include <sbmpo_ros/model.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>

using namespace sbmpo;
namespace model {

    #define NUM_SAMPLES 11
    Control controls[NUM_SAMPLES];

    bool initialize(Planner &planner) {
        // TODO
        return true;
    }

    float cost(const sbmpo::Node& current, const sbmpo::Node &next) {
        // TODO
        return 0;
    }

    float heuristic(const sbmpo::Node& current, const sbmpo::State &goal) {
        //TODO
        return 0;
    }

    bool is_valid(const sbmpo::State& state) {
        // TODO
        return true;
    }

    bool next_state(Node &node, const Planner &planner, const int n) {

        const Node& parent = planner.buffer[node.parent_id];

        const float v0 = parent.control[0];
        const float u0 = parent.control[1];

        const float gx = planner.options.state_info[0].goal_value;
        const float gy = planner.options.state_info[1].goal_value;

        // Generate set of controls
        Control control = controls[n];
        node.control = control;
        const float v = control[0];
        const float u = control[1];

        const float sample_time = planner.options.sample_time;
        const float sample_time_increment = planner.options.sample_time_increment;

        float &x = node.state[0];
        float &y = node.state[1];
        float &w = node.state[2];

        for (float t = 0; t < sample_time; t += sample_time_increment) {

            // New yaw
            const float nw = w + u * sample_time_increment;

            // New position
            const float nx = x + cosf(nw) * v * sample_time_increment;
            const float ny = y + sinf(nw) * v * sample_time_increment;

            // Check if valid
            if (!is_valid({nx, ny, nw}))
                return false;

            // Update positions if valid
            w = nw; x = nx; y = ny;
        }

        // Yaw angle wrapping, w = (-pi, pi]
        if (w >= 2.0*M_PI || w < 0.0f)
            w += w < 0.0f ? 2.0*M_PI : -2.0*M_PI;

        return true;
    }

    template<class T> void send_external(T &obj) {}

    template<class T> void get_external(T &obj) {}

    template void get_external<grid_map::GridMap>(grid_map::GridMap&);



}