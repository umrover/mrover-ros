#include <iostream>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <motors_group.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "brushed_test_bridge");
    ros::NodeHandle nh;

    auto brushlessController = std::make_unique<mrover::BrushedController>(nh, "jetson", "joint_b");

    ros::Rate rate(1);
    while (ros::ok()) {
        brushlessController->setDesiredThrottle(0.8);
        rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
