#include <ros/ros.h>

#include "joint_de.hpp"
#include <memory.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "arm_translator_bridge");
    ros::NodeHandle nh;

    [[maybe_unused]] auto jointDEHandler = std::make_unique<mrover::JointDE>(nh);

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}