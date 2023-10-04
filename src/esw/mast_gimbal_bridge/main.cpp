#include <ros/ros.h>

#include <motors_manager.hpp>

std::unique_ptr<MotorsManager> mastGimbalManager;
std::vector<std::string> mastGimbalNames =
        {"mast_gimbal_x", "mast_gimbal_y"};

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "mast_gimbal_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    mastGimbalManager = std::make_unique<MotorsManager>(nh, "mast_gimbal", mastGimbalNames);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}