#include "messaging.hpp"
#include <mrover/CAN.h>
#include <ros/ros.h>

void processCANData(const mrover::CAN::ConstPtr& msg);

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "imu_arduino_bridge");
    ros::NodeHandle nh;

    ros::Subscriber CANSubscriber = nh.subscribe<mrover::CAN>("can/imu_arduino/in", 1, processCANData);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void processCANData(const mrover::CAN::ConstPtr& msg) {
    assert(msg->source == "imu_arduino");
    assert(msg->destination == "jetson");
    ROS_INFO("%f cool stuff",msg->data[0]);
}
