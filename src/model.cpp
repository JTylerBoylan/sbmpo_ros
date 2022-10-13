#include <sbmpo_ros/model.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/CircleIterator.hpp>

using namespace sbmpo;
namespace model {

    typedef Eigen::Vector2d Vector;
    typedef std::pair<Vector, float> Obstacle;

    const Vector bounds[2] = {
        {-1.0, -1.0},
        {6.0, 6.0}  
    };

    #define BODY_WIDTH 0.1
    #define BODY_HEIGHT 0.1
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

    #define NUM_SAMPLES 11
    Control controls[NUM_SAMPLES];

    static Planner * planner;

    bool initialize(Planner &_planner) {
        ROS_INFO("External Initialization");
        const std::string halton_path = ros::package::getPath("sbmpo_ros") + "/data/halton.csv";
        ROS_INFO("Finding Halton Data in '%s'", halton_path.c_str());
        std::map<std::string, std::vector<float>> halton_csv = read_csv(halton_path);
        ROS_INFO("Found Data. Adding to controls");
        ROS_INFO("Control samples (v, u):");
        for (int i = 0; i < NUM_SAMPLES; i++) {
            for (auto iter = halton_csv.begin(); iter != halton_csv.end(); ++iter) {
                const float sample = iter->second[i];
                controls[i].push_back(sample);
            }
            ROS_INFO("  - (%.2f, %.2f)", controls[i][0], controls[i][1]);
        }
        planner = &_planner;
        ROS_INFO("Initialized");
        return true;
    }

    // G-score increment for a given sample
    float cost(const sbmpo::Node& current, const sbmpo::Node &next) {
        return abs(current.control[0]) * planner->options.sample_time;
    }

    // H-score for a given node 
    float heuristic(const sbmpo::Node& current, const sbmpo::State& goal) {
        const float dx = goal[0] - current.state[0];
        const float dy = goal[1] - current.state[1];
        return sqrtf(dx*dx + dy*dy);
    }

    bool is_valid(const sbmpo::State& state) {

        // Find collision between body and obstacle or 
        const Vector origin(state[0], state[1]);
        const Eigen::Matrix2d rotation = Eigen::Rotation2Dd(state[2]).toRotationMatrix();
        for (int bd = 0; bd < 4; bd++) {

            const Vector x1 = rotation * body[bd] + origin;
            const Vector x2 = rotation * body[(bd+1)%4] + origin;
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

    bool is_goal(const sbmpo::State& state) {
        float sum = 0.0f;
        for (int i = 0; i < state.size(); i++) {
            const StateInfo &info = planner->options.state_info[i];
            const float val = state[i];
            if (info.goal_radius != -1.0f)
                sum += powf((state[i] - info.goal_value) / info.goal_radius, 2.0f);
        }
        return sum <= 1.0f;
    }

    bool next_state(Node &node, const int n) {

        const Node& parent = planner->buffer[node.parent_id];

        // Generate set of controls
        Control control = controls[n];
        node.control = control;
        const float v = control[0];
        const float u = control[1];

        //ROS_INFO("Sample %d: (v = %.2f, u = %.2f)", n, v, u);

        const float sample_time = planner->options.sample_time;
        const float sample_time_increment = planner->options.sample_time_increment;

        float &x = node.state[0];
        float &y = node.state[1];
        float &w = node.state[2];

        for (float t = 0; t < sample_time; t += sample_time_increment) {

            sbmpo::State next(3);

            // New position
            next[0] = x + cosf(w) * v * sample_time_increment;
            next[1] = y + sinf(w) * v * sample_time_increment;

            // New yaw
            next[2]= w + u * sample_time_increment;

            // Check if valid
            if (!is_valid(next))
                return false;

            // Update positions if valid
            x = next[0]; y = next[1]; w = next[2];
        }

        // Yaw angle wrapping, w = (-pi, pi]
        if (w >= 2.0*M_PI || w < 0.0f)
            w += w < 0.0f ? 2.0*M_PI : -2.0*M_PI;

        //ROS_INFO("Add to queue: [%d](%.2f, %.2f, %.2f)", node.id, node.state[0], node.state[1], node.state[2]);

        return true;
    }

    template<class T> void send_external(T &obj) {}

    template<class T> void get_external(T &obj) {
        grid_map::GridMap &map = obj;
        map.get("elevation").setConstant(0.0);
        map.get("obstacle").setConstant(0.0);
        for (int ob = 0; ob < NUM_OBSTACLES; ob++)
            for (auto iter = grid_map::CircleIterator(map, obstacles[ob].first, obstacles[ob].second); 
                    !iter.isPastEnd(); ++iter)
                map.at("obstacle", *iter) = 1.0;
    }

    template void get_external<grid_map::GridMap>(grid_map::GridMap&);

}