#include "can_device.hpp"
#include "messaging.hpp"
#include <ros/ros.h>

#include <mrover/CAN.h>
#include <mrover/LED.h>

std::unique_ptr<mrover::CanDevice> ledCanDevice;

void changeLED(const mrover::LED::ConstPtr& msg);

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "led_hw_bridge");
    ros::NodeHandle nh;

    ledCanDevice = std::make_unique<mrover::CanDevice>(nh, "jetson", "pdlb");

    ros::Subscriber changeLEDSubscriber = nh.subscribe<mrover::LED>("led", 1, changeLED);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void changeLED(const mrover::LED::ConstPtr& msg) {
    mrover::LEDInfo ledInfo{};
    ledInfo.red = msg->red;
    ledInfo.green = msg->green;
    ledInfo.blue = msg->blue;
    ledInfo.blinking = msg->is_blinking;
    ledCanDevice->publish_message(mrover::InBoundPDLBMessage{mrover::LEDCommand{.led_info = ledInfo}});
}