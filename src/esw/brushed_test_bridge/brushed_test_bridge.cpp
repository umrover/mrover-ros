#include <ros/ros.h>

#include <can_manager.hpp>
#include <iostream>
#include <motors_manager.hpp>
#include <units/units.hpp>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "brushed_test_bridge");
    ros::NodeHandle nh;

    [[maybe_unused]] auto brushlessController = std::make_unique<mrover::BrushedController>(nh, "jetson", "devboard");

    brushlessController->set_desired_throttle(0.5);

    // Enter the ROS event loop
    ros::spinOnce();

    return EXIT_SUCCESS;
}
