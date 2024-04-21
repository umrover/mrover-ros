#include "messaging.hpp"
#include <mrover/CAN.h>
#include <ros/ros.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/RelativeHumidity.h>

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

void processMessage(mrover::DirtData const& message) {
    sensor_msgs::Temperature temperature;
    temperature.temperature = message.temperature;
    sensor_msgs::RelativeHumidity humidity;
    humidity.relative_humidity = message.humidity;
    temperatureDataPublisher->publish(temperature);
    humidityDataPublisher->publish(humidity);
}

void processCANData(const mrover::CAN::ConstPtr& msg) {
    assert(msg->source == "dirt_sensor_arduino");
    assert(msg->destination == "jetson");

    mrover::OutBoundSASensorMessage const& message = *reinterpret_cast<mrover::OutBoundSASensorMessage const*>(msg->data.data());

    // This calls the correct process function based on the current value of the alternative
    std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
}
