#include "messaging.hpp"
#include <motors_manager.hpp>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

std::unique_ptr<mrover::CanDevice> armLaserCanDevice;

std::vector<std::string> armNames{"joint_a", "joint_b", "joint_c", "joint_de_0", "joint_de_1", "finger", "gripper"};

bool armLaserCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    armLaserCanDevice->publish_message(mrover::InBoundPDLBMessage{mrover::ArmLaserCommand{.enable = static_cast<bool>(req.data)}});
    res.success = true;
    res.message = "DONE";
    return true;
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "arm_bridge");
    ros::NodeHandle nh;

    armLaserCanDevice = std::make_unique<mrover::CanDevice>(nh, "jetson", "pdlb");

    [[maybe_unused]] auto armManager = std::make_unique<mrover::MotorsManager>(nh, "arm", armNames);

    nh.advertiseService("enable_arm_laser", armLaserCallback);

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}