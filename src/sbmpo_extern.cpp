#include <sbmpo_ros/sbmpo_extern.hpp>
#include <grid_map_core/GridMap.hpp>

namespace sbmpo {

    static grid_map::GridMap * map;

    // Parameters

    float max_velocity = 1.0f;
    float max_rotation = M_PI / 60.0;

    float body_mass = 10.0f; // kg
    float body_moment = 100.0f; // kg m^2

    float drag_force = 10.0f; // J/m

    float forward_factor = 1.0;
    float reverse_factor = 2.0;

    float gravity_force = 10.0f; // J/m

    float uphill_factor = 1.0f;
    float downhill_factor = 0.25f;

    float acceleration_factor = 1.0f;
    float decceleration_factor = 0.25f;
    float rotational_factor = 1.0f;

    float heat_transfer = 5e-10f;

    float hot_factor = 0.0f;
    float cool_factor = 100.0f;

    float desired_temperature = 300.0f;
    float initial_temperature = 350.0f;

    float map2temp_a = -520.0f;
    float map2temp_b = 26.0f;
    float map2temp_c = 400.0f;

    // G-score increment for a given sample
    float dg(const float dt, const float v, const float u, const float dv, const float du, 
                        const float dz, const float T) {

        const float direction_factor = v > 0 ? forward_factor : reverse_factor;
        const float potential_factor = dz > 0 ? uphill_factor : downhill_factor;
        const float accel_factor = dv > 0 ? acceleration_factor : decceleration_factor;

        const float dT = T - desired_temperature;
        const float temperature_factor = dT > 0 ? hot_factor : cool_factor;

        const float drag_energy = drag_force * abs(v) * dt * direction_factor;
        const float potential_energy = gravity_force * abs(dz) * potential_factor;
        const float kinetic_energy = body_mass * abs(v) * abs(dv) * accel_factor;
        const float rotation_energy = body_moment * abs(u) * abs(du) * rotational_factor;

        const float temperature_energy = abs(dT) * temperature_factor;

        return drag_energy + potential_energy + kinetic_energy + rotation_energy + temperature_energy;
    }

    // H-score for a given node
    float h(const float x, const float y, const float w, const float gx, const float gy) {
        const float dx = gx - x;
        const float dy = gy - y;
        float dw = atan2f(dy,dx) - w;
        if (dw > M_PI || dw <= -M_PI)
            dw += dw > M_PI ? -2.0f*M_PI : 2.0*M_PI;
        const float dt = sqrtf(dx*dx + dy*dy)/max_velocity + abs(dw)/max_rotation;
        const float dz = map->atPosition("elevation", grid_map::Position(gx, gy)) -
                    map->atPosition("elevation", grid_map::Position(x, y));
        return dg(dt, max_velocity, max_rotation, 0.0f, 0.0f, dz, desired_temperature);
    }

    bool evaluate(Node &node, const Planner &planner) {

        const Node& parent = planner.buffer[node.parent_id];

        const float v0 = parent.control[0];
        const float u0 = parent.control[1];

        const float gx = planner.options.state_info[0].goal_value[0];
        const float gy = planner.options.state_info[0].goal_value[1];

        const float v = node.control[0];
        const float u = node.control[1];

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

        // Get temperature from map
        // T_K = a*T_map^2 + b*T_map + c
        const float T_map = map->atPosition("temperature", position);
        const float T_K = map2temp_a * powf(T_map, 2.0f) + map2temp_b * T_map + map2temp_c;

        // m*c*dT/dt = e*o*A*(Ts^4 - T^4) --> dT = (e*o*A)/(m*c) * (Ts^4 - T^4) * dt

        // Increase body temperature
        T += heat_transfer * (powf(T_K, 4.0f) - powf(T, 4.0f)) * sample_time;

        // Increase g score
        g += dg(sample_time, v, u, v - v0, u - u0, z1 - z0, T);

        f = g + h(x, y, w, gx, gy);

        return true;
    }

    template<class T> void send_external(T &obj) {
        map = &obj;
    }

    template void send_external<grid_map::GridMap>(grid_map::GridMap &obj);

}