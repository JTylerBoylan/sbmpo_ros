#include <sbmpo_ros/sbmpo_extern.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>

using namespace sbmpo;
namespace sbmpo_ext {

    #define NUM_SAMPLES 11
    Control controls[NUM_SAMPLES];

    bool initialize(Planner &planner) {
        // TODO
        return true;
    }

    // G-score increment for a given sample
    float dg(const float dt, const float v, const float u) {
        // TODO
        return 0;
    }

    // H-score for a given node
    float h(const float x, const float y, const float w, const float gx, const float gy) {
        //TODO
        return 0;
    }

    bool isValid(const float x, const float y, const float w) {
        // TODO
        return true;
    }

    bool evaluate(Node &node, const Planner &planner, const int n) {

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

        float &f = node.heuristic[0];
        float &g = node.heuristic[1];

        for (float t = 0; t < sample_time; t += sample_time_increment) {

            // New yaw
            const float nw = w + u * sample_time_increment;

            // New position
            const float nx = x + cosf(nw) * v * sample_time_increment;
            const float ny = y + sinf(nw) * v * sample_time_increment;

            // Check if valid
            if (!isValid(nx, ny, nw))
                return false;

            // Update positions if valid
            w = nw; x = nx; y = ny;
        }

        // Yaw angle wrapping, w = (-pi, pi]
        if (w >= 2.0*M_PI || w < 0.0f)
            w += w < 0.0f ? 2.0*M_PI : -2.0*M_PI;

        // Increase g score
        g += dg(sample_time, v, u);

        f = g + h(x, y, w, gx, gy);

        return true;
    }

    template<class T> void send_external(T &obj) {}

    template<class T> void get_external(T &obj) {}

}