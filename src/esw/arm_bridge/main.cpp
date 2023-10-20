#include <ros/ros.h>

#include <motors_manager.hpp>

std::vector<std::string> armNames{"joint_a", "joint_b", "joint_c", "joint_de_0", "joint_de_1", "finger", "gripper"};

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "arm_bridge");
    ros::NodeHandle nh;

    [[maybe_unused]] auto armManager = std::make_unique<mrover::MotorsManager>(nh, "arm", armNames);

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}
