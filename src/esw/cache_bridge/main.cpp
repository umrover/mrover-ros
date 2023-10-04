#include <ros/ros.h>

#include <motors_manager.hpp>

std::unique_ptr<MotorsManager> cacheManager;
std::vector<std::string> cacheNames =
        {"cache"};

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "cache_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    cacheManager = std::make_unique<MotorsManager>(nh, "cache", cacheNames);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}