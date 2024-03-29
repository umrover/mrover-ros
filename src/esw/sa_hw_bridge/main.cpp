#include "can_device.hpp"

#include <motors_group.hpp>
#include <ros/ros.h>

auto main(int argc, char** argv) -> int {
    // Initialize the ROS node
    ros::init(argc, argv, "sa_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    [[maybe_unused]] auto SAManager = std::make_unique<mrover::MotorsGroup>(nh, "sa_hw");
    // Enter the ROS event loop
    ros::spin();

    return 0;
}