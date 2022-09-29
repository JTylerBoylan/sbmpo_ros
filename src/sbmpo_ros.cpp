#include <sbmpo_ros/sbmpo_ros.hpp>

namespace sbmpo {

    void configure(Planner &planner, const ros::NodeHandle handle) {

        ROS_INFO("Configuring");

        handle.getParam("max_iterations", planner.options.max_iterations);
        handle.getParam("max_generations", planner.options.max_generations);
        handle.getParam("sample_size", planner.options.sample_size);
        handle.getParam("sample_type", planner.options.sample_type);

        if (planner.options.sample_type == "input") {
            XmlRpc::XmlRpcValue samples;
            handle.getParam("samples", samples);
            generateSampleList(planner.options.sample_list, samples);
        }

        std::map<std::string, float*> variable_map;

        XmlRpc::XmlRpcValue state_list, control_list;
        handle.getParam("states", state_list);
        handle.getParam("controls", control_list);

        for (int s = 0; s < state_list.size(); s++) {
            XmlRpc::XmlRpcValue state = state_list[s];
            for (auto param = state.begin(); param != state.end(); ++param) {
                ROS_INFO("%s", param->first.c_str());
            }
        }


    }

    void generateSampleList(SampleList &sample_list, const XmlRpc::XmlRpcValue &samples) {

    }

}