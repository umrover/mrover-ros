#include <ros/ros.h>

#include <motors_manager.hpp>

std::vector<std::string> cacheNames{"cache"};

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "cache_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    [[maybe_unused]] auto cacheManager = std::make_unique<mrover::MotorsManager>(nh, "cache", cacheNames);

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}