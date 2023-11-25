#include <controller.hpp>
#include <ros/ros.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "joint_de_bridge");
    ros::NodeHandle nh;

    // TODO - the idea is to have this mutliple controllers. then it should listen to joint_de_pitch and joint_de_yaw

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}