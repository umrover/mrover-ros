#include "std_srvs/SetBool.h"
#include <ros/ros.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/Temperature.h>
#include <unordered_map>

void tempCallback(const ros::TimerEvent&);
void humidityCallback(const ros::TimerEvent&);

ros::Publisher tempDataPublisher;
ros::Publisher humidityDataPublisher;


int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "sa_sensor_bridge");
    ros::NodeHandle nh;

    tempDataPublisher = nh.advertise<sensor_msgs::Temperature>("sa_temp_data", 1);
    humidityDataPublisher = nh.advertise<sensor_msgs::RelativeHumidity>("sa_humidity_data", 1);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void tempCallback(const ros::TimerEvent&) {
    sensor_msgs::Temperature tempData;
    tempData.temperature = 0; // TODO
    tempDataPublisher.publish(tempData);
}

void humidityCallback(const ros::TimerEvent&) {
    sensor_msgs::RelativeHumidity humidityData;
    humidityData.relative_humidity = 0; // TODO
    humidityDataPublisher.publish(humidityData);
}