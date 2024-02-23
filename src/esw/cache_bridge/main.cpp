#include <ros/ros.h>

#include <motors_group.hpp>

auto main(int argc, char** argv) -> int {
    // Initialize the ROS node
    ros::init(argc, argv, "cache_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    [[maybe_unused]] auto cacheManager = std::make_unique<mrover::MotorsGroup>(nh, "cache");

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}