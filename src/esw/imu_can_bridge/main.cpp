#include "messaging.hpp"
#include <mrover/CAN.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/RelativeHumidity.h>

std::unique_ptr<ros::Publisher> imuDataPublisher;

void processCANData(const mrover::CAN::ConstPtr& msg);

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "imu_can_bridge");
    ros::NodeHandle nh;

    imuDataPublisher = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::Imu>("imu_data", 1));

    ros::Subscriber CANSubscriber = nh.subscribe<mrover::CAN>("can/imu_arduino/in", 1, processCANData);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void processMessage(mrover::DirtData const& message) {
    sensor_msgs::Imu imu;
    // update so that the assignments below are correct
    imu.orientation = message.orientation;
    imu.velocity = message.velocity;
    imu.acceleration = message.acceleration;
    imuDataPublisher->publish(imu);
}

void processCANData(const mrover::CAN::ConstPtr& msg) {
    assert(msg->source == "imu_arduino");
    assert(msg->destination == "jetson");

    mrover::OutBoundSASensorMessage const& message = *reinterpret_cast<mrover::OutBoundSASensorMessage const*>(msg->data.data());

    // This calls the correct process function based on the current value of the alternative
    std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
}
