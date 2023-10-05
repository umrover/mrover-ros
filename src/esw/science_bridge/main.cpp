#include <ros/ros.h>
#include "std_srvs/SetBool.h"
#include <unordered_map>

std::unordered_map<std::string, int> device_name_to_index = {
    {"heater_0", 0},
    {"heater_1", 1},
    {"heater_2", 2},
    {"heater_3", 3},
    {"heater_4", 4},
    {"heater_5", 5},
    {"white_led_0", 6},
    {"white_led_1", 7},
    {"white_led_2", 8},
    {"uv_led_0", 9},
    {"uv_led_1", 10},
    {"uv_led_2", 11}
};

bool serviceCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res, int device_id) {
    ROS_INFO("TODO - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "science_bridge");
    ros::NodeHandle nh;

    for (auto const& [deviceName, deviceID] : device_name_to_index) {
        // Advertise services and set the callback using a lambda function
        nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
            "science_enable_" + deviceName,
            [deviceID](std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
                return serviceCallback(req, res, deviceID);
            }
        );
    }
    

    // Enter the ROS event loop
    ros::spin();

    return 0;
}