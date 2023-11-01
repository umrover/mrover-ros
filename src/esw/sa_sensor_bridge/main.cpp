#include "std_srvs/SetBool.h"
#include <ros/ros.h>
#include <unordered_map>

void tempCallback(const ros::TimerEvent&);
void humiditiCallback(const ros::TimerEvent&);

ros::publisher tempDataPublisher;
ros::publisher humidityDataPublisher;


int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "sa_sensor_bridge");
    ros::NodeHandle nh;

    // TODO

    tempDataPublisher = nh.advertise<sensor_msgs::Temperature>("drive_joint_data", 1);
    humidityDataPublisher = nh.advertise<sensor_msgs::Temperature>("drive_controller_data", 1);


    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void tempCallback(const ros::TimerEvent&) {
    // TODO

    senor_msgs::Temperature tempData;
    tempDataPublisher.publish(tempData);
}

void humiditiCallback(const ros::TimerEvent&) {
    // TODO

    sensor_msgs::RelativeHumidity humidityData;
    humidityDataPublisher.publish(humidityData);
}