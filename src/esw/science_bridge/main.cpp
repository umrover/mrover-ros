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

bool serviceCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    ROS_INFO("TODO - request for device_id TODO %i. Value: %s", 0, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}


int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "science_bridge");
    ros::NodeHandle nh;

    for (const auto& entry : device_name_to_index) {
        ros::ServiceServer service_server = nh.advertiseService(
            "science_enable_" + entry.first,
            serviceCallback
        );
    }

    // Enter the ROS event loop
    ros::spin();

    return 0;
}
