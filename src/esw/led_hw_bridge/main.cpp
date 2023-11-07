#include "can_fd_bus.hpp"

#include <ros/ros.h>

#include <mrover/CAN.h>
#include <mrover/LED.h>

void changeLED(const mrover::LED::ConstPtr& msg);

ros::Publisher CANPublisher;

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "led_hw_bridge");
    ros::NodeHandle nh;

    CANPublisher = nh.advertise<mrover::CAN>("can_requests", 1);
    ros::Subscriber changeLEDSubscriber = nh.subscribe<mrover::LED>("led", 1, changeLED);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void changeLED(const mrover::LED::ConstPtr& msg) {
    mrover::CAN CANRequest;
    CANRequest.bus = 0; // TODO
    CANRequest.message_id = 0;
    uint8_t data = msg->blue | (msg->red & 0b1) | (msg->green << 1) | (msg->blue << 2) | (msg->is_blinking << 3);
    CANRequest.data.push_back(data);
    CANPublisher.publish(CANRequest);
}