#include <ros/ros.h>
#include "std_srvs/SetBool.h"
#include <unordered_map>

bool heater0Callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    int device_id = 0;
    ROS_INFO("Heater 0 - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

bool heater1Callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    int device_id = 1;
    ROS_INFO("Heater 1 - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

bool heater2Callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    int device_id = 2;
    ROS_INFO("Heater 2 - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

bool heater3Callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    int device_id = 3;
    ROS_INFO("Heater 3 - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

bool heater4Callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    int device_id = 4;
    ROS_INFO("Heater 4 - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

bool heater5Callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    int device_id = 5;
    ROS_INFO("Heater 5 - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

bool whiteLED0Callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    int device_id = 6;
    ROS_INFO("White LED 0 - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

bool whiteLED1Callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    int device_id = 7;
    ROS_INFO("White LED 1 - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

bool whiteLED2Callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    int device_id = 8;
    ROS_INFO("White LED 2 - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

bool uvLED0Callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    int device_id = 9;
    ROS_INFO("UV LED 0 - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

bool uvLED1Callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    int device_id = 10;
    ROS_INFO("UV LED 1 - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

bool uvLED2Callback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    int device_id = 11;
    ROS_INFO("UV LED 2 - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "science_bridge");
    ros::NodeHandle nh;

    nh.advertiseService("science_enable_heater_0", heater0Callback);
    nh.advertiseService("science_enable_heater_1", heater1Callback);
    nh.advertiseService("science_enable_heater_2", heater2Callback);
    nh.advertiseService("science_enable_heater_3", heater3Callback);
    nh.advertiseService("science_enable_heater_4", heater4Callback);
    nh.advertiseService("science_enable_heater_5", heater5Callback);
    nh.advertiseService("science_enable_white_led_0", whiteLED0Callback);
    nh.advertiseService("science_enable_white_led_1", whiteLED1Callback);
    nh.advertiseService("science_enable_white_led_2", whiteLED2Callback);
    nh.advertiseService("science_enable_uv_led_0", uvLED0Callback);
    nh.advertiseService("science_enable_uv_led_1", uvLED1Callback);
    nh.advertiseService("science_enable_uv_led_2", uvLED2Callback);
    // Enter the ROS event loop
    ros::spin();

    return 0;
}
