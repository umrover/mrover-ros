#include <ros/ros.h>
#include <mrover/CAN.h>

void sendCAN(const mrover::CAN::ConstPtr& msg)

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "can_bridge");
    ros::NodeHandle nh;

    // Subscribe to the ROS topic for arm commands
    ros::Subscriber CANRequestSubscriber = n->subscribe<mrover::CAN>("can_requests", 1, sendCAN);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void sendCAN(const mrover::CAN::ConstPtr& msg) {
    // TODO
}