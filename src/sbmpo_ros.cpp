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

        std::map<std::string, float*> variable_map;

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
                    parseStateGrid(param->second, planner.grid);
                } else if (section == "control") {
                    parseStateControl(param->second, state_info);
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
                Range goal;
                for (int i = 0; i < param->second.size(); i++)
                    goal[i] = float(double(param->second[i]));
                if (param->second.size() < 2)
                    goal[1] = goal[0];
                info.goal_value = goal;
                ROS_INFO("  Goal range: [%.2f %.2f]", goal[0], goal[1]);
            } else if (name == "range") {
                Range range;
                for (int i = 0; i < 2; i++)
                    range[i] = float(double(param->second[i]));
                info.range = range;
                ROS_INFO("  Value range: [%.2f %.2f]", range[0], range[1]);
            } else if (name == "overflow") {
                std::string overflow_type = std::string(param->second);
                info.overflow = toOverflowType(overflow_type);
                ROS_INFO("  Overflow type: %s (%i)", overflow_type.c_str(), info.overflow);
            } else {
                ROS_ERROR("Unknown state value parameter: %s", name.c_str());
            }
        }
        if (info.goal_value[1] < info.range[0] || info.goal_value[0] > info.range[1])
            info.defined_goal = false;
        ROS_INFO("  Defined goal: %s", info.defined_goal ? "true" : "false");
    }

    void parseStateGrid(const XmlRpc::XmlRpcValue &value, ImplicitGrid &grid) {
        for (auto param = value.begin(); param != value.end(); ++param) {
            std::string name = param->first;
            if (name == "active") {
                grid.active.push_back(bool(param->second));
                ROS_INFO("  Gridded: %s", grid.active.back() ? "true" : "false");
            } else if (name == "resolution") {
                grid.resolution.push_back(float(double(param->second)));
                ROS_INFO("  Grid resolution: %.2f", grid.resolution.back());
            } else if (name == "size") {
                grid.size.push_back(int(param->second));
                ROS_INFO("  Grid size: %i", grid.size.back());
            } else {
                ROS_ERROR("Unknown state implicit_grid parameter: %s", name.c_str());
            }
        }
    }

    void parseStateControl(const XmlRpc::XmlRpcValue &control, StateInfo &info) {

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

}