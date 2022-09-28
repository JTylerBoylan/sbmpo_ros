#include <sbmpo_ros/Planner.hpp>

using namespace sbmpo;
using namespace Eigen;

// Constructor
Planner::Planner() {

    // Set parameters
    max_iterations = 1000000; // 156 MB per million iterations @ 3 samples / iteration (B = 52*S*I)
    max_generations = 100;
    sample_size = 3;

    velocities = {0.75, 1.0, 0.75}; // m/s
    rotations = {M_PI / 60.0, 0.0, -M_PI / 60.0}; // rad/s
    max_velocity = 1.0;
    max_rotation = M_PI / 60.0;

    goal_radius = 3.0; // m

    sample_time = 10.0; // s
    sample_time_increment = 1.0;

    body_mass = 10; // kg
    body_moment = 100; // kg m^2

    drag_force = 10; // J/m

    forward_factor = 1.0;
    reverse_factor = 2.0;

    gravity_force = 10; // J/m

    uphill_factor = 1.0;
    downhill_factor = 0.25;

    acceleration_factor = 1.0;
    decceleration_factor = 0.25;
    rotational_factor = 1.0;

    // Create buffer
    buffer = new Node[max_iterations*sample_size + 1];
}

// Destructor
Planner::~Planner() {
    delete[] buffer;
}

// Run path finding simulation
void Planner::run(const grid_map::GridMap &map, const Odometry &odom, const Point &point) {

    // Starting conditions
    const Point position = odom.pose.pose.position;
    const geometry_msgs::Quaternion orientation = odom.pose.pose.orientation;
    const Twist twist = odom.twist.twist;

    // Save grid
    this->map = &map;
    this->goal = point;

    // Initialize start node
    Node start;
    
    start.i = 0;
    start.p = -1;
    start.G = 0;
    start.t = 0.0f;

    start.x = float(position.x);
    start.y = float(position.y);
    start.w = 2.0f * atan2f(Vector3f(orientation.x, orientation.y, orientation.z).norm(), orientation.w);
    
    // Angle inverted if orientation z < 0
    if (orientation.z < 0) 
        start.w = 2.0*M_PI - start.w;

    start.v = twist.linear.x;
    start.u = twist.angular.z;

    start.g = 0.0f;
    start.f = h(start);

    // Insert start node into buffer
    buffer[0] = start;

    // Create comparator function for priority queue, ordered from highest to lowest f-score
    const std::function<bool(int, int)> comp = [this](int a, int b) { return buffer[a].f > buffer[b].f; };

    // Initialize priority queue
    std::priority_queue<int, std::vector<int>, const std::function<bool(int, int)>> queue =
        std::priority_queue<int, std::vector<int>, const std::function<bool(int, int)>>(comp);

    // Reset high
    high = 0;

    // Track best node
    int best = 0;

    // Begin iterations
    for (int iter = 0; iter < max_iterations; iter++) {

        // Get node from buffer
        Node node = buffer[best];

        // Goal check
        if (powf(node.x - goal.x, 2.0) + powf(node.y - goal.y, 2.0) <= goal_radius*goal_radius)
            break;

        // Generation check
        if (node.G >= max_generations)
            break;

        // Run sampling
        for (int n = 0; n < sample_size; n++) {
            const int response = sample(node, n);
            if (response)
                queue.push(response);
        }

        // Update highest node
        high += sample_size;

        // Check if any nodes left
        if (queue.empty())
            break;

        // Get node with lowest f score
        best = queue.top();

        // Remove from priority queue
        queue.pop();

    }

    // Iterations ended

    // Generate best path
    path.clear();
    for (int i = best; i != -1; i = buffer[i].p)
        path.push_back(i);

    // Reverse so path is ordered start -> best
    std::reverse(path.begin(),path.end());

    // Print path389.65
    ROS_INFO("--- PATH ---");
    for (int idx : path) {
        const Node node = buffer[idx];
        ROS_INFO("[%i] (x: %.2f, y: %.2f, w: %.2f) (v: %.2f, u: %.2f) (g: %.2f, h: %.2f, f: %.2f)",
                idx, node.x, node.y, node.w, node.v, node.u, node.g, h(node), node.f);
    }

    // End of run function
}

// Get best path
void Planner::getPath(Path& path) {
    for (int i : this->path) {
        PoseStamped pose;
        if (map->isInside(grid_map::Position(buffer[i].x, buffer[i].y))) {
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = map->getFrameId();
            pose.pose = toPose(buffer[i]);
            path.poses.push_back(pose);
        }
    }
}

// Get next twist
void Planner::getTwist(Twist& twist) {
    double v = 0.0, u = 0.0;
    if (path.size() > 1) {
        const Node next = buffer[path[1]];
        v = next.v;
        u = next.u;
    }
    twist.linear.x = v;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = u;
}

// Get all nodes in buffer as a pose
void Planner::getAllPoses(PoseArray &poses) {
    for (int i = 0; i < high; i++)
        if (map->isInside(grid_map::Position(buffer[i].x, buffer[i].y)))
            poses.poses.push_back(toPose(buffer[i]));
}

// H-score for a given node
float Planner::h(const Node &node) {
    const float dx = goal.x - node.x;
    const float dy = goal.y - node.y;
    float dw = atan2f(dy,dx) - node.w;
    if (dw > M_PI || dw <= -M_PI)
        dw += dw > M_PI ? -2.0f*M_PI : 2.0*M_PI;
    const float dt = sqrtf(dx*dx + dy*dy)/max_velocity + abs(dw)/max_rotation;
    const float dz = map->atPosition("elevation", grid_map::Position(goal.x, goal.y)) -
                map->atPosition("elevation", grid_map::Position(node.x, node.y));
    return dg(dt, max_velocity, max_rotation, 0.0f, 0.0f, dz);
}

// G-score increment for a given sample
float Planner::dg(const float dt, const float v, const float u, const float dv, const float du, 
                    const float dz) {

    const float direction_factor = v > 0 ? forward_factor : reverse_factor;
    const float potential_factor = dz > 0 ? uphill_factor : downhill_factor;
    const float accel_factor = dv > 0 ? acceleration_factor : decceleration_factor;

    const float drag_energy = drag_force * abs(v) * dt * direction_factor;
    const float potential_energy = gravity_force * abs(dz) * potential_factor;
    const float kinetic_energy = body_mass * abs(v) * abs(dv) * accel_factor;
    const float rotation_energy = body_moment * abs(u) * abs(du) * rotational_factor;

    return drag_energy + potential_energy + kinetic_energy + rotation_energy;
}

// Sampling
int Planner::sample(const Node& node, const int n) {

    // Save position
    const grid_map::Position position0(node.x, node.y);

    // Copy node values
    float x = node.x;
    float y = node.y;
    float w = node.w;
    float g = node.g;

    // Get sample velocity and rotation
    const float v = velocities[n];
    const float u = rotations[n];

    // Sampling time
    grid_map::Position position;
    for (float t = 0; t < sample_time; t += sample_time_increment) {

        // New yaw
        w += u * sample_time_increment;

        // New position
        x += cosf(w) * v * sample_time_increment;
        y += sinf(w) * v * sample_time_increment;

        // Bounds check
        position = grid_map::Position(x,y);
        if (!(map->isInside(position)))
            return 0; // return invalid response
    }

    // Yaw angle wrapping, w = (-pi, pi]
    if (w > M_PI || w <= -M_PI)
        w += w > M_PI ? -2.0*M_PI : 2.0*M_PI;

    // Get elevation data from map
    const float z0 = map->atPosition("elevation", position0);
    const float z1 = map->atPosition("elevation", position);

    // Increase g score
    g += dg(sample_time, v, u, v - node.v, u - node.u, z1 - z0);

    // Calculate node index (valid response)
    const int res = high + n + 1;

    // Copy into buffer
    buffer[res] = {x, y, w, v, u, g, 0.0f, res, node.i, node.t + sample_time, node.G+1};

    // Calculate F score
    buffer[res].f = g + h(buffer[res]);

    return res;
}

Pose Planner::toPose(const Node& node) {

    const float x = node.x;
    const float y = node.y;
    const float w = node.w;

    const grid_map::Position pos(x, y);
    const float z = map->atPosition("elevation", pos);

    Vector3d forward(2.0 * cos(w), 2.0 * sin(w), 0.0);
    const grid_map::Position fwd(x + forward.x(), y + forward.y());
    forward.z() = map->isInside(fwd) ? map->atPosition("elevation", fwd) - z : 0.0;

    Vector3d lateral(2.0 * cos(w + M_PI_2), 2.0 * sin(w + M_PI_2), 0.0);
    const grid_map::Position lat(x + lateral.x(), y + lateral.y());
    lateral.z() = map->isInside(lat) ? map->atPosition("elevation", lat) - z : 0.0;

    Vector3d normal = forward.cross(lateral).normalized();

    const double sin_w2 = sin(w / 2.0);
    const double cos_w2 = cos(w / 2.0);

    geometry_msgs::Pose p;
    p.position.x = x;
    p.position.y = y;
    p.position.z = z;
    p.orientation.x = normal.x() * sin_w2;
    p.orientation.y = normal.y() * sin_w2;
    p.orientation.z = normal.z() * sin_w2;
    p.orientation.w = cos_w2;

    return p;
}