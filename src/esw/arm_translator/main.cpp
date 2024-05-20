#include <ros/ros.h>

#include "arm_translator.hpp"

auto main(int argc, char** argv) -> int {
    // Initialize the ROS node
    ros::init(argc, argv, "arm_translator_bridge");
    ros::NodeHandle nh;

    [[maybe_unused]] auto armTranslator = std::make_unique<mrover::ArmTranslator>(nh);

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}