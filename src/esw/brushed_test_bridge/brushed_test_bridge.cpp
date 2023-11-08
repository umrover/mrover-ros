#include <iostream>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <motors_manager.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "brushed_test_bridge");
    ros::NodeHandle nh;

    auto brushlessController = std::make_unique<mrover::BrushedController>(nh, "jetson", "devboard");

    ros::Rate rate(1);
    while (ros::ok()) {
        brushlessController->setDesiredThrottle(0.5);
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
