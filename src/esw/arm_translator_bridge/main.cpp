#include <ros/ros.h>

#include "arm_translator.hpp"
#include <memory.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "arm_translator_bridge");
    ros::NodeHandle nh;

    [[maybe_unused]] auto armTranslator = std::make_unique<mrover::ArmTranslator>(nh);

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}