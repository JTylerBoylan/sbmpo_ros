#include <sbmpo_ros/sbmpo_ros.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

int main (int argc, char ** argv) {
    
    ros::init(argc, argv, "sbmpo");
    ros::NodeHandle handle("~");

    sbmpo::Planner planner;
    sbmpo::configure(planner, handle);

    std::string map_topic,
                path_topic,
                poses_topic;
    handle.getParam("map_topic", map_topic);
    handle.getParam("path_topic", path_topic);
    handle.getParam("poses_topic", poses_topic);

    bool publish_all;
    handle.getParam("publish_all", publish_all);

    float publish_rate;
    handle.getParam("publish_rate", publish_rate);

    grid_map::GridMap map;
    bool map_init = false;

    auto mapCallback = [&](const grid_map_msgs::GridMapConstPtr &map_msg) {
        if (!map_init) map_init = true;
        grid_map::GridMapRosConverter::fromMessage(*map_msg, map);
        sbmpo::send_external<grid_map::GridMap>(map);
    };

    ros::Subscriber map_sub = handle.subscribe<grid_map_msgs::GridMap>(map_topic, 10, mapCallback);

    ros::Publisher path_pub = handle.advertise<nav_msgs::Path>(path_topic, 10, true);
    ros::Publisher poses_pub = handle.advertise<geometry_msgs::PoseArray>(poses_topic, 1, true);

    float t, tt = 0.0f;
    int n = 0;

    ros::Rate rate(publish_rate);
    while (handle.ok()) {

        ros::spinOnce();

        if (!map_init)
            continue;

        ROS_INFO("Running planner...");

        clock_t cstart, cend;

        cstart = clock();

        sbmpo::run(planner);

        cend = clock();

        n++;
        t = float(cend - cstart) / float(CLOCKS_PER_SEC) * 1000.0f;
        tt += t;

        geometry_msgs::PoseArray all_poses;
        nav_msgs::Path path;
        ros::Time ts = ros::Time::now();

        path.header.frame_id = map.getFrameId();
        path.header.stamp = ts;

        sbmpo::getPath(path, planner, map);
        path_pub.publish(path);

        ROS_INFO("Planner Path published. \tComputing Time: %.2f ms (Avg: %.2f ms)", t, tt / n);

        if (publish_all) {
            all_poses.header.frame_id = map.getFrameId();
            all_poses.header.stamp = ts;

            sbmpo::getAllPoses(all_poses, planner, map);
            poses_pub.publish(all_poses);
        }

        rate.sleep();
    }

    sbmpo::deconstruct(planner);
    
}