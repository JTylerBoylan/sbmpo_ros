#include <sbmpo_ros/sbmpo_extern.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>

namespace sbmpo {

    typedef Eigen::Vector2d Vector;
    typedef std::pair<Vector, float> Obstacle;

    const Vector bounds[2] = {
        {-1.0, -1.0},
        {6.0, 6.0}  
    };

    #define BODY_WIDTH 0.5
    #define BODY_HEIGHT 0.5
    const Vector body[4] = {
        {0.0, 0.0},
        {0.0, BODY_HEIGHT},
        {BODY_WIDTH, BODY_HEIGHT},
        {BODY_WIDTH, 0.0}
    };

    #define NUM_OBSTACLES 3
    const Obstacle obstacles[NUM_OBSTACLES] = {
        {{3.1, 1.2}, 0.5},
        {{3.5, 3.7}, 0.5},
        {{1.0, 0.5}, 0.5}
    };


    // Generate Halton samples
    Control generateHaltonSamples(const int n, const unsigned int seed, const ControlInfoList &info);

    // Generate random samples
    Control generateRandomSamples(const int n, const unsigned int seed, const ControlInfoList &info);

    // G-score increment for a given sample
    float dg(const float dt, const float v, const float u) {
        // !-- TODO --!
        return 1.0;
    }

    // H-score for a given node
    float h(const float x, const float y, const float w, const float gx, const float gy) {
        // !-- TODO --!
        return 0.0;
    }

    bool isValid(const float x, const float y, const float w) {

        // Find collision between body and obstacle or 
        const Vector origin(x,y);
        const Eigen::Rotation2Df rotation(w);
        for (int bd = 0; bd < 4; bd++) {

            const Vector x1 = body[bd] + origin;
            const Vector x2 = body[(bd+1)%4] + origin;
            const Vector v = x2 - x1;
            const Vector del = v.normalized();

            // Bounds check
            Vector lowleft = x1 - bounds[0];
            Vector upright = bounds[1] - x1;
            if (lowleft.x() < 0 || lowleft.y() < 0 ||
                upright.x() < 0 || upright.y() < 0)
                return false;

            // Collision check
            for (int ob = 0; ob < NUM_OBSTACLES; ob++) {

                const Vector obstacle = obstacles[ob].first;
                const float threshold = obstacles[ob].second;
                const Vector A = obstacle - x1;
                const float d = A.x() * del.y() - A.y() * del.x();

                if (abs(d) > threshold)
                    continue;

                const Vector B = obstacle - x2;
                const float dx1 = del.dot(x1);
                const float dx2 = del.dot(x2);
                const float dob = del.dot(obstacle);

                if (A.norm() < threshold ||
                    B.norm() < threshold ||
                    (dx1 < dob &&
                     dob < dx2) ||
                    (dx2 < dob &&
                     dob < dx1))
                    return false;

            }
        }

        return true;
    }

    bool evaluate(Node &node, const Planner &planner, const int n) {

        const Node& parent = planner.buffer[node.parent_id];

        const float v0 = parent.control[0];
        const float u0 = parent.control[1];

        const float gx = planner.options.state_info[0].goal_value;
        const float gy = planner.options.state_info[1].goal_value;

        // Generate set of controls
        //Control control = controls[n];
        //Control control = generateRandomSamples(node.control.size(), node.id, planner.options.control_info);
        Control control = generateHaltonSamples(node.control.size(), node.id + 1000, planner.options.control_info);
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
                break;

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

    template<class T> void get_external(T &obj) {
        grid_map::GridMap &map = obj;
        map.get("elevation").setConstant(0.0);
        map.get("obstacle").setConstant(0.0);
        for (int ob = 0; ob < NUM_OBSTACLES; ob++)
            for (auto iter = grid_map::CircleIterator(map, obstacles[ob].first, obstacles[ob].second); 
                    iter.isPastEnd(); ++iter)
                map.at("obstacle", *iter) = 1.0;
    }

    template void get_external<grid_map::GridMap>(grid_map::GridMap&);


    /*
        Sample Generation Functions
    */

    // Stores primes used in Halton sampling
    static const int primes[10] = {2, 3, 5, 7, 11, 13, 17, 19, 23, 29}; 
    Control generateHaltonSamples(const int n, const unsigned int seed, const ControlInfoList &info) {
        Control control;
        double x;
        int k, p, num;
        for (int j = 0; j < n; j++) {
            x = 0.0, k = 1, p = primes[j], num = seed;
            while (num > 0) {
                x += double(num % p) / pow(p, k++);
                num /= p;
            }
            const float lower_bound = info[j].range_min;
            const float upper_bound = info[j].range_max;
            x *= upper_bound - lower_bound;
            x += lower_bound;
            control.push_back(x);
        }
        return control;
    }

    static bool seeded_rand = false;
    Control generateRandomSamples(const int n, const unsigned int seed, const ControlInfoList &info) {
        if (!seeded_rand) {
            srand(seed);
            seeded_rand = true;
        }
        Control samples;
        for (int j = 0; j < n; j++) {
            float x = double(rand()) / RAND_MAX;
            const float lower_bound = info[j].range_min;
            const float upper_bound = info[j].range_max;
            x *= upper_bound - lower_bound;
            x += lower_bound;
            samples.push_back(x);
        }
        return samples;
    }

}