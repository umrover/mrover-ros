#include "messaging.hpp"
#include <mrover/CAN.h>
#include <ros/ros.h>

std::unique_ptr<ros::Publisher> temperatureDataPublisher;
std::unique_ptr<ros::Publisher> humidityDataPublisher;

void processCANData(const mrover::CAN::ConstPtr& msg);

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "dirt_sensor_arduino_bridge");
    ros::NodeHandle nh;

    temperatureDataPublisher = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::Temperature>("sa_temp_data", 1));
    humidityDataPublisher = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::RelativeHumidity>("sa_humidity_data", 1));

    ros::Subscriber CANSubscriber = nh.subscribe<mrover::CAN>("can/dirt_sensor_arduino/in", 1, processCANData);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void processMessage(sensor_msgs::Temperature const& message) {
    sensor_msgs::Temperature temperature;
    temperature = message;
    spectralDataPublisher->publish(temperature);
}

void processMessage(sensor_msgs::RelativeHumidity const& message) {
    sensor_msgs::RelativeHumidity humidity;
    humidity = message;
    spectralDataPublisher->publish(humidity);
}

void processCANData(const mrover::CAN::ConstPtr& msg) {
    assert(msg->source == "dirt_sensor_arduino");
    assert(msg->destination == "jetson");
    ROS_INFO("data: %f",msg->data);

    mrover::OutBoundSASensorMessage const& message = *reinterpret_cast<mrover::OutBoundSASensorMessage const*>(msg->data);

    // This calls the correct process function based on the current value of the alternative
    std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
}
