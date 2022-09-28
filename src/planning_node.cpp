#include <ros/ros.h>
#include <sbmpo_ros/Planner.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <geometry_msgs/PointStamped.h>

using namespace grid_map;
using namespace sbmpo;

int main (int argc, char ** argv) {

    ros::init(argc, argv, "planner");

    ros::NodeHandle node("~");

    bool publish_all,
        print_path;

    std::string map_topic,
                odom_topic,
                goal_topic,
                path_topic,
                poses_topic,
                twist_topic,
                frame_id;

    float publish_rate;

    node.getParam("publish_all", publish_all);
    node.getParam("print_path", print_path);

    node.getParam("map_topic", map_topic);
    node.getParam("odom_topic", odom_topic);
    node.getParam("goal_topic", goal_topic);
    node.getParam("path_topic", path_topic);
    node.getParam("poses_topic", poses_topic);
    node.getParam("twist_topic", twist_topic);
    node.getParam("frame_id", frame_id);
    node.getParam("publish_rate", publish_rate);

    GridMap grid;

    nav_msgs::Odometry odom;
    Point goal;

    Planner planner;

    ros::Publisher pub_poses = node.advertise<PoseArray>(poses_topic, 1, true);
    ros::Publisher pub_path = node.advertise<nav_msgs::Path>(path_topic, 1, true);
    ros::Publisher pub_twist = node.advertise<Twist>(twist_topic, 1, true);

    bool map_init = false, odom_init = false, goal_init = false;

    float t, tt = 0;
    int n = 0;

    auto mapUpdate = [&](const grid_map_msgs::GridMap::ConstPtr& grid_msg) {
        if (!map_init) map_init = true;
        GridMapRosConverter::fromMessage(*grid_msg, grid);
    };

    auto odomUpdate = [&](const nav_msgs::Odometry::ConstPtr& odom_msg) {
        if (!odom_init) odom_init = true;
        odom = *odom_msg;

    };

    auto goalUpdate = [&](const PointStamped::ConstPtr& goal_msg) {
        if (!goal_init) goal_init = true;
        goal = goal_msg->point;
    };

    ros::Subscriber map_sub = node.subscribe<grid_map_msgs::GridMap>(map_topic, 1, mapUpdate);
    ros::Subscriber odom_sub = node.subscribe<nav_msgs::Odometry>(odom_topic, 1, odomUpdate);
    ros::Subscriber goal_sub = node.subscribe<PointStamped>(goal_topic, 1, goalUpdate);

    ros::Rate rate(publish_rate);
    while (node.ok()) {

        ros::spinOnce();

        if (!map_init || !goal_init || !odom_init)
            continue;

        clock_t cstart, cend;

        cstart = clock();

        planner.run(grid, odom, goal);

        geometry_msgs::PoseArray all_poses;
        geometry_msgs::Twist twist;
        nav_msgs::Path path;
        ros::Time ts = ros::Time::now();

        path.header.frame_id = frame_id;
        path.header.stamp = ts;

        planner.getPath(path);
        pub_path.publish(path);

        cend = clock();
        
        n++;
        t = float(cend - cstart) / float(CLOCKS_PER_SEC) * 1000.0f;
        tt += t;

        ROS_INFO("Planner Path published. (TS: %f) \tComputing Time: %.2f ms (Avg: %.2f ms)", ts.toSec(), t, tt / n);

        planner.getTwist(twist);
        pub_twist.publish(twist);

        if (publish_all) {
            all_poses.header.frame_id = frame_id;
            all_poses.header.stamp = ts;

            planner.getAllPoses(all_poses);
            pub_poses.publish(all_poses);
        }

        rate.sleep();
    }

    return 0;

}