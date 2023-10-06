#include "CanNode.hpp"
#include <ros/ros.h>
#include <mrover/CAN.h>

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
    uint8_t bus = msg->bus;
    uint16_t id = msg->message_id;
    size_t dataLength = (msg->data).size();
    std::vector<uint8_t> data = msg->data;

    node.sendFrame(bus, id, dataLength, data);
}
