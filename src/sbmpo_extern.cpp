#include <sbmpo_ros/sbmpo_extern.hpp>
#include <grid_map_core/GridMap.hpp>

namespace sbmpo {

    static grid_map::GridMap * map;

    // Parameters

    const float max_velocity = 1.0f;
    const float max_rotation = M_PI / 60.0;

    const float body_mass = 10.0f; // kg
    const float body_moment = 100.0f; // kg m^2

    const float drag_force = 10.0f; // J/m

    const float forward_factor = 1.0;
    const float reverse_factor = 2.0;

    const float gravity_force = 10.0f; // J/m

    const float uphill_factor = 1.0f;
    const float downhill_factor = 0.25f;

    const float acceleration_factor = 1.0f;
    const float decceleration_factor = 0.25f;
    const float rotational_factor = 1.0f;

    const float heat_transfer = 5e-10f;

    const float hot_factor = 0.0f;
    const float cool_factor = 100.0f;

    const float desired_temperature = 300.0f;
    const float initial_temperature = 350.0f;

    const float map2temp_a = -520.0f;
    const float map2temp_b = 26.0f;
    const float map2temp_c = 400.0f;

    const Control controls[3] = {
        {0.75f, -0.0524f},
        {1.0f, 0.0f},
        {0.75f, 0.0524f}
    };

    // G-score increment for a given sample
    float dg(const float dt, const float v1, const float v2, const float u1, const float u2,
                        const float z1, const float z2, const float T) {

        const float dv = v2 - v1;
        const float du = u2 - u1;
        const float dz = z2 - z1;

        const float kinetic_factor = v1 > 0 ? 1 : -1;
        const float rotation_factor = u1 > 0 ? 1 : -1;
        const float direction_factor = v2 > 0 ? forward_factor : reverse_factor;
        const float potential_factor = dz > 0 ? uphill_factor : downhill_factor;
        const float accel_factor = dv > 0 ? acceleration_factor : decceleration_factor;

        const float dT = T - desired_temperature;
        const float temperature_factor = dT > 0 ? hot_factor : cool_factor;

        const float drag_energy = drag_force * abs(v2) * dt * direction_factor;
        const float potential_energy = gravity_force * abs(dz) * potential_factor;
        const float kinetic_energy = 0.5 * body_mass * abs(v2*v2 - kinetic_factor*v1*v1);
        const float rotation_energy = 0.5 * body_moment * abs(u2*u2 - rotation_factor*u1*u1);

        const float temperature_energy = abs(dT) * temperature_factor;

        return drag_energy + potential_energy + kinetic_energy + rotation_energy + temperature_energy;
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
        return dg(dt, v, max_velocity, u, max_rotation, z1, z1, desired_temperature);
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

            // New yaw
            const float nw = w + u * sample_time_increment;

            // New position
            const float nx = x + cosf(nw) * v * sample_time_increment;
            const float ny = y + sinf(nw) * v * sample_time_increment;

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
        const float T_K = map2temp_a * powf(T_map, 2.0f) + map2temp_b * T_map + map2temp_c;

        // m*c*dT/dt = e*o*A*(Ts^4 - T^4) --> dT = (e*o*A)/(m*c) * (Ts^4 - T^4) * dt

        // Increase body temperature
        T += heat_transfer * (powf(T_K, 4.0f) - powf(T, 4.0f)) * sample_time;

        // Increase g score
        g += dg(sample_time, v0, v, u0, u, z0, z, T);

        f = g + h(x, y, w, v, u, gx, gy);

        return true;
    }

    template<class T> void send_external(T &obj) {
        map = &obj;
    }

    template void send_external<grid_map::GridMap>(grid_map::GridMap &obj);

}