#include <ros/ros.h>

#include <iostream>
#include <motors_manager.hpp>
#include <units/units.hpp>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "brushless_test_bridge");
    ros::NodeHandle nh;
    ROS_INFO("Running");

    [[maybe_unused]] auto brushlessController = std::make_unique<mrover::BrushlessController>(nh, "jetson", "devboard");

    // Enter the ROS event loop
    brushlessController->SetStop();
    ROS_INFO("Sent stop command");
    ros::Rate rate(1);
    while (ros::ok()) {
        brushlessController->setDesiredVelocity(mrover::RadiansPerSecond{1.0f});
        ROS_INFO("Sent velocity command to Moteus");
        rate.sleep();
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}
