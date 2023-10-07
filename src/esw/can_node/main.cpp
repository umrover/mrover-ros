#include "CanNode.hpp"
#include <mrover/CAN.h>
#include <ros/ros.h>

void handleMessage(const mrover::CAN::ConstPtr& msg);

CanNode node;

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_node");
    ros::NodeHandle nh;

    ros::Subscriber CanSubscriber = nh.subscribe<mrover::CAN>("can_requests", 1, handleMessage);

    ros::spin();

    return 0;
}

void handleMessage(const mrover::CAN::ConstPtr& msg) {
    node.set_bus(msg->bus);
    node.set_frame_id(msg->message_id);
    node.set_frame_data((msg->data).size(), msg->data);

    node.send_frame();
}
