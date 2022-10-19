#include <sbmpo_ros/sbmpo_extern.hpp>
#include <grid_map_core/GridMap.hpp>

namespace sbmpo {

    static grid_map::GridMap * map;

    // Parameters

    const float max_velocity = 0.5f;
    const float max_rotation = M_PI / 60.0;

    const float body_mass = 2.5f; // kg
    const float body_moment = 0.0625f; // kg m^2

    const float drag_force = 10.0f; // J/m

    const float forward_factor = 1.0;
    const float reverse_factor = 2.0;

    const float gravity_force = body_mass * 1.62; // J/m

    const float uphill_factor = 1.0f;
    const float downhill_factor = 0.25f;

    const float acceleration_factor = 1.0f;
    const float decceleration_factor = 0.25f;
    const float rotational_factor = 1.0f;

    const float heat_transfer = body_mass * 900; // J/K
    const float motor_gen = 0; // W
    const float robot_area = 0.08; //m^2
    const float epsilon_g = 0.95;
    const float epsilon_out = 1.0;
    const float sigma = 0.0000000567;
    const float T_out4 = 53.1441;
    const float emissivity = 0.15;
    const float k_leg = 170;
    const float legs_area = 0.0005;
    const float L_leg = 0.2;

    const float hot_factor = 0.0f;
    const float cool_factor = 100.0f;

    const float desired_temperature = 300.0f;
    const float initial_temperature = 300.0f;

    const float map2temp_a = -520.0f;
    const float map2temp_b = 26.0f;
    const float map2temp_c = 400.0f;

    const Control controls[3] = {
        {0.5f, -0.0524f},
        {0.5f, 0.0f},
        {0.5f, 0.0524f}
    };

    // G-score increment for a given sample
    float dg(const float dt, const float v1, const float v2, const float u1, const float u2,
                        const float z1, const float z2, const float T, const float T_map) {

        const float dv = v2 - v1;
        const float du = u2 - u1;
        const float dz = z2 - z1;

        const float direction_factor = v2 > 0 ? forward_factor : reverse_factor;
        const float potential_factor = dz > 0 ? uphill_factor : downhill_factor;
        const float accel_factor = dv > 0 ? acceleration_factor : decceleration_factor;

        const float dT = T - desired_temperature;
        const float temperature_factor = dT > 0 ? hot_factor : cool_factor;

        const float drag_energy = drag_force * abs(v2) * dt * direction_factor;
        const float potential_energy = gravity_force * abs(dz) * potential_factor;
        const float kinetic_energy = body_mass * abs(dv) * abs(v2);
        const float rotation_energy = body_moment * abs(du) * abs(u2);

        const float ground_factor = T_map > 300 ? 0.001 : 0.01;
        const float ground_energy = powf(T_map - 300, 2.0f) * ground_factor;
        const float temperature_energy = abs(dT) * temperature_factor;

        return drag_energy + potential_energy + kinetic_energy + rotation_energy + temperature_energy + ground_energy;
    }

    // H-score for a given node
    float h(const float x, const float y, const float w, const float v, const float u, const float gx, const float gy) {
        const float dx = gx - x;
        const float dy = gy - y;
        float dw = atan2f(dy,dx) - w;
        if (dw > M_PI || dw <= -M_PI)
            dw += dw > M_PI ? -2.0f*M_PI : 2.0*M_PI;
        const float dt = sqrtf(dx*dx + dy*dy)/max_velocity + abs(dw)/max_rotation;
        const float z2 = map->atPosition("elevation", grid_map::Position(gx, gy));
        const float z1 = map->atPosition("elevation", grid_map::Position(x, y));
        return dg(dt, v, max_velocity, u, max_rotation, z1, z1, desired_temperature, 300);
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
        float &T = node.state[3];

        float &f = node.heuristic[0];
        float &g = node.heuristic[1];

        const grid_map::Position position0(x,y);

        grid_map::Position position;
        for (float t = 0; t < sample_time; t += sample_time_increment) {

            // New position
            const float nx = x + cosf(w) * v * sample_time_increment;
            const float ny = y + sinf(w) * v * sample_time_increment;

            // New yaw
            const float nw = w + u * sample_time_increment;

            // Bounds check
            position = grid_map::Position(nx,ny);
            if (!(map->isInside(position)))
                return false; // return invalid response

            // Update positions if valid
            w = nw; x = nx; y = ny;
        }

        // Yaw angle wrapping, w = (-pi, pi]
        if (w >= 2.0*M_PI || w < 0.0f)
            w += w < 0.0f ? 2.0*M_PI : -2.0*M_PI;

        // Get elevation data from map
        const float z0 = map->atPosition("elevation", position0);
        const float z = map->atPosition("elevation", position);

        // Get temperature from map
        // T_K = a*T_map^2 + b*T_map + c
        const float T_map = map->atPosition("temperature", position);
        const float T_inv = (1.0f - T_map);
        const float T_K = map2temp_a * powf(T_map, 2.0f) + map2temp_b * T_map + map2temp_c;

        // m*c*dT/dt = e*o*A*(Ts^4 - T^4) --> dT = (e*o*A)/(m*c) * (Ts^4 - T^4) * dt

        // Increase body temperature
        T += sample_time / heat_transfer * (motor_gen + T_inv * robot_area + epsilon_g*sigma*robot_area*powf(T_K, 4.0f) + epsilon_out*sigma*robot_area*T_out4
                    - 2.0*emissivity*sigma*robot_area*powf(T, 4.0f) - k_leg * legs_area / L_leg * (T - T_K));

        // Increase g score
        g += dg(sample_time, v0, v, u0, u, z0, z, T, T_K);

        f = g + h(x, y, w, v, u, gx, gy);

        return true;
    }

    template<class T> void send_external(T &obj) {
        map = &obj;
    }

    template void send_external<grid_map::GridMap>(grid_map::GridMap &obj);

}