#include "messaging.hpp"
#include <motors_group.hpp>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

std::unique_ptr<mrover::CanDevice> armLaserCanDevice;

auto armLaserCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) -> bool {
    armLaserCanDevice->publish_message(mrover::InBoundPDLBMessage{mrover::ArmLaserCommand{.enable = static_cast<bool>(req.data)}});
    res.success = true;
    res.message = "DONE";
    return true;
}

auto main(int argc, char** argv) -> int {
    // Initialize the ROS node
    ros::init(argc, argv, "arm_hw_bridge");
    ros::NodeHandle nh;

    armLaserCanDevice = std::make_unique<mrover::CanDevice>(nh, "jetson", "pdlb");

    [[maybe_unused]] auto armManager = std::make_unique<mrover::MotorsGroup>(nh, "arm_hw");

    ros::ServiceServer armLaserService = nh.advertiseService("enable_arm_laser", armLaserCallback);

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}