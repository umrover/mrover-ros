#include <ros/ros.h>
#include <mrover/LED.h>
#include <mrover/CAN.h>

void changeLED(const mrover::LED::ConstPtr& msg);

ros::Publisher CANPublisher;

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "led_hw_bridge");
    ros::NodeHandle nh;

    CANPublisher = n->advertise<mrover::CAN>("can_requests", 1);
    // Subscribe to the ROS topic for arm commands
    ros::Subscriber moveArmSubscriber = n->subscribe<mrover::LED>("led_hw_bridge", 1, changeLED);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void changeLED(const mrover::LED::ConstPtr& msg) {
    mrover::CAN CANRequest;
    CANRequest.name = "led_hw";
    CANRequest.data = (msg->red) | (msg->green << 1) | (msg->blue << 2) | (msg->is_blinking << 3);
    CANPublisher.publish(CANRequest);
}