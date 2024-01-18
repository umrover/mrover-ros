#include "messaging.hpp"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <mrover/HeaterData.h>
#include <stdexcept>

int main(int argc, char** argv) {
    ros::init(argc, argv, "science_test_bridge");
    ros::NodeHandle nh;

    /* Arm Laser Test */
    ros::service::waitForService("enable_arm_laser");
    ros::ServiceClient service_client = nh.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("enable_arm_laser");

    ROS_INFO("****BEGIN ARM LASER TEST****");

    try {
        ROS_INFO("Enabling arm laser...");
        test_arm_laser_enable(true);

        ROS_INFO("Disabling arm laser...");
        test_arm_laser_enable(false);
    } catch (std::exception const& e) {
        ROS_ERROR("Arm laser test FAILED: %s", e.what());
        return 1;
    }

    ROS_INFO("****END ARM LASER TEST****");

    /* Auton LED Test */
    

    return 0;
}

void test_arm_laser_enable(bool enable) {
    std_srvs::SetBool::Request request;
        request.data = enable;

    if (auto resp_code = std_srvs::SetBool::Response resp; service_client.call(request, resp); resp_code) {
        ROS_INFO("Response: %s", resp.message.c_str());
    } else {
        ROS_ERROR("Failed to call service");
        throw std::runtime_error(resp_code);
    }
}