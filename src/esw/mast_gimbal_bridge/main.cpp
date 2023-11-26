#include <ros/ros.h>

#include <motors_group.hpp>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "mast_gimbal_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    [[maybe_unused]] auto mastGimbalManager = std::make_unique<mrover::MotorsGroup>(nh, "mast_gimbal");

    // Enter the ROS event loop
    ros::spin();

    return 0;
}
