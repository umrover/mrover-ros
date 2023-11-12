#include <ros/ros.h>

#include <iostream>
#include <motors_manager.hpp>
#include <units/units.hpp>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "brushless_test_bridge");
    ros::NodeHandle nh;
    ROS_INFO("Running");

    //[[maybe_unused]] auto brushlessController = std::make_unique<mrover::BrushlessController>(nh, "jetson", "test_brushless_controller");
    [[maybe_unused]] auto brushlessController = std::make_unique<mrover::BrushlessController>(nh, "laptop", "test_brushless_controller");
    brushlessController->set_desired_velocity(mrover::RadiansPerSecond{1.0f});
    ROS_INFO("Sent velocity command to Moteus");
    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}
