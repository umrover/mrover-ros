#include <ros/ros.h>

#include <motors_group.hpp>

auto main(int argc, char** argv) -> int {
    // Initialize the ROS node
    ros::init(argc, argv, "mast_gimbal_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    std::ignore = std::make_unique<mrover::MotorsGroup>(nh, "mast_gimbal");

    // Enter the ROS event loop
    ros::spin();

    return 0;
}
