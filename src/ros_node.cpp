#include <sbmpo_ros/sbmpo_ros.hpp>

int main (int argc, char ** argv) {
    
    ros::init(argc, argv, "sbmpo");
    ros::NodeHandle handle("~");

    sbmpo::Planner planner;
    sbmpo::configure(planner, handle);

    ros::Rate rate(10);
    while (handle.ok()) {

        // Run planner

        rate.sleep();
        ros::spinOnce();

    }

    sbmpo::deconstruct(planner);
    
}