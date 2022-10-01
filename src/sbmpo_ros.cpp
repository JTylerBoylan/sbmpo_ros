#include <sbmpo_ros/sbmpo_ros.hpp>

namespace sbmpo {

    void configure(Planner &planner, const ros::NodeHandle handle) {

        ROS_INFO("Configuring Planner...");

        handle.getParam("max_iterations", planner.options.max_iterations);
        handle.getParam("max_generations", planner.options.max_generations);
        handle.getParam("sample_size", planner.options.sample_size);

        ROS_INFO("Max Iterations: %d", planner.options.max_iterations);
        ROS_INFO("Max generations: %d", planner.options.max_generations);
        ROS_INFO("Sample size: %d", planner.options.sample_size);

        std::string sample_type;
        handle.getParam("sample_type", sample_type);
        planner.options.sample_type = toSampleType(sample_type);

        ROS_INFO("Sample type: %s (%i)", sample_type.c_str(), planner.options.sample_type);

        if (planner.options.sample_type == SampleType::INPUT) {
            XmlRpc::XmlRpcValue samples;
            handle.getParam("samples", samples);
            ROS_INFO("Samples:");
            generateSampleList(planner.options.sample_list, samples);
        }

        handle.getParam("sample_time", planner.options.sample_time);
        handle.getParam("sample_time_increment", planner.options.sample_time_increment);

        ROS_INFO("Sample Time: %.2f", planner.options.sample_time);
        ROS_INFO("Sample Time Increment: %.2f", planner.options.sample_time_increment);

        XmlRpc::XmlRpcValue state_list, control_list;
        handle.getParam("states", state_list);
        handle.getParam("controls", control_list);

        ROS_INFO("Configuring States");

        for (int s = 0; s < state_list.size(); s++) {
            XmlRpc::XmlRpcValue state = state_list[s];
            StateInfo state_info;
            ROS_INFO("State %i:", s);
            for (auto param = state.begin(); param != state.end(); ++param) {
                std::string section = param->first;
                if (section == "name") {
                    parseStateName(param->second, state_info);
                } else if (section == "value") {
                    parseStateValue(param->second, state_info);
                } else if (section == "implicit_grid") {
                    parseStateGrid(param->second, state_info);
                } else {
                    ROS_ERROR("Unknown state parameter: %s", section.c_str());
                }
            }
            planner.options.state_info.push_back(state_info);
        }

        ROS_INFO("Configuring Controls");

        for (int c = 0; c < control_list.size(); c++) {
            XmlRpc::XmlRpcValue control = control_list[c];
            ControlInfo control_info;
            ROS_INFO("Control %i:", c);
            for (auto param = control.begin(); param != control.end(); ++param) {
                std::string section = param->first;
                if (section == "name") {
                    parseControlName(param->second, control_info);
                } else if (section == "value") {
                    parseControlValue(param->second, control_info);
                } else {
                    ROS_ERROR("Unknown control parameter: %s", section.c_str());
                }
            }
            planner.options.control_info.push_back(control_info);
        }

        ROS_INFO("Initializing Planner");

        initialize(planner);

    }

    void generateSampleList(SampleList &sample_list, const XmlRpc::XmlRpcValue &samples) {
        for (int i = 0; i < samples.size(); i++) {
            Control control;
            XmlRpc::XmlRpcValue sample = samples[i];
            std::string list_str;
            for (int j = 0; j < sample.size(); j++) {
                control.push_back(float(double(sample[j])));
                list_str += " " + std::to_string(control.back());
            }
            sample_list.push_back(control);
            ROS_INFO("- [%s ]", list_str.c_str());
        }
    }

    void parseStateName(const XmlRpc::XmlRpcValue &value, StateInfo &info) {
        info.name = std::string(value);
        ROS_INFO("  Name: %s", info.name.c_str());
    }

    void parseStateValue(const XmlRpc::XmlRpcValue &value, StateInfo &info) {
        for (auto param = value.begin(); param != value.end(); ++param) {
            std::string name = param->first;
            if (name == "initial") {
                info.initial_value = float(double(param->second));
                ROS_INFO("  Initial value: %.2f", info.initial_value);
            } else if (name == "goal") {
                if (param->second.getType() == XmlRpc::XmlRpcValue::TypeBoolean)
                    info.defined_goal = bool(param->second);
                else {
                    Range goal;
                    for (int i = 0; i < param->second.size(); i++)
                        goal[i] = float(double(param->second[i]));
                    if (param->second.size() < 2)
                        goal[1] = goal[0];
                    info.goal_value = goal;
                    ROS_INFO("  Goal range: [%.2f %.2f]", goal[0], goal[1]);
                    info.goal_avg = (goal[0] + goal[1]) / 2.0f;
                    ROS_INFO("  Goal average: %.2f", info.goal_avg);
                    info.defined_goal = true;
                }
                ROS_INFO("  Defined goal: %s", info.defined_goal ? "true" : "false");
            } else {
                ROS_ERROR("Unknown state value parameter: %s", name.c_str());
            }
        }
    }

    void parseStateGrid(const XmlRpc::XmlRpcValue &value, StateInfo &info) {
        for (auto param = value.begin(); param != value.end(); ++param) {
            std::string name = param->first;
            if (name == "active") {
                info.grid = bool(param->second);
                ROS_INFO("  Gridded: %s", info.grid ? "true" : "false");
            } else if (name == "resolution") {
                info.grid_resolution = float(double(param->second));
                ROS_INFO("  Grid resolution: %.2f", info.grid_resolution);
            } else if (name == "size") {
                info.grid_size = int(param->second);
                ROS_INFO("  Grid size: %i", info.grid_size);
            } else {
                ROS_ERROR("Unknown state implicit_grid parameter: %s", name.c_str());
            }
        }
    }

    void parseControlName(const XmlRpc::XmlRpcValue &value, ControlInfo &info) {
        info.name = std::string(value);
        ROS_INFO("  Name: %s", info.name.c_str());
    }

    void parseControlValue(const XmlRpc::XmlRpcValue &value, ControlInfo &info) {
        for (auto param = value.begin(); param != value.end(); ++param) {
            std::string name = param->first;
            if (name == "initial") {
                info.initial_value = float(double(param->second));
                ROS_INFO("  Initial value: %.2f", info.initial_value);
            } else if (name == "range") {
                Range range;
                for (int i = 0; i < 2; i++)
                    range[i] = float(double(param->second[i]));
                info.range = range;
                ROS_INFO("  Value range: [%.2f %.2f]", range[0], range[1]);
            } else {
                ROS_ERROR("Unknown control value parameter: %s", name.c_str());
            }
        }
    }

    void getPath(nav_msgs::Path& path, const Planner &planner, const grid_map::GridMap &map) {
        for (int i : planner.results.path) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = map.getFrameId();
            pose.pose = toPose(planner.buffer[i], map);
            path.poses.push_back(pose);
        }
    }

    void getAllPoses(geometry_msgs::PoseArray &poses, const Planner &planner, const grid_map::GridMap &map) {
        for (int i = 0; i < planner.results.high; i++)
            if (planner.buffer[i].id != INVALID_INDEX)
                poses.poses.push_back(toPose(planner.buffer[i], map));
    }

    geometry_msgs::Pose toPose(const Node &node, const grid_map::GridMap &map) {
        const float x = node.state[0];
        const float y = node.state[1];
        const float w = node.state[2];

        const grid_map::Position og(x, y);
        const float z = map.atPosition("elevation", og);

        const int n = 4;

        Eigen::Vector3d vectors[n];
        for (int v = 0; v < n; v++) {
            const double rot = 2.0*M_PI*double(v)/double(n);
            vectors[v] = Eigen::Vector3d(cos(w + rot), sin(w + rot), 0.0);
            const grid_map::Position pos(x + vectors[v].x(), y + vectors[v].y());
            vectors[v].z() = map.isInside(pos) ? map.atPosition("elevation", pos) - z : 0.0;
        }

        // Normalization from "3D Math Primer for Graphics and Game Development"
        Eigen::Vector3d normal(0.0, 0.0, 0.0);
        Eigen::Vector3d prev = vectors[n-1];
        for (int i = 0; i < n; i++) {
            const Eigen::Vector3d curr = vectors[i];
            normal.x() += (prev.z() + curr.z()) * (prev.y() - curr.y());
            normal.y() += (prev.x() + curr.x()) * (prev.z() - curr.z());
            normal.z() += (prev.y() + curr.y()) * (prev.x() - curr.x());
            prev = curr;
        }

        normal.normalize();

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

}