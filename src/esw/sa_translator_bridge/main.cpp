#include <ros/ros.h>

#include "sa_translator.hpp"
#include <memory.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "sa_translator_bridge");
    ros::NodeHandle nh;

    [[maybe_unused]] auto saTranslator = std::make_unique<mrover::SATranslator>(nh);

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}