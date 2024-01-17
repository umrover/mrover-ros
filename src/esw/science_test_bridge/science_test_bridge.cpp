#include "messaging.hpp"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <mrover/HeaterData.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "science_test_bridge");
    ros::NodeHandle nh;

    ros::service::waitForService("/science_change_heater_auto_shutoff_state");

    try {
        ros::ServiceClient service_client = nh.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("/science_change_heater_auto_shutoff_state");

        std_srvs::SetBool::Request request;
        request.data = false;


        if (std_srvs::SetBool::Response resp; service_client.call(request, resp)) {
            ROS_INFO("Response: %s", resp.message.c_str());
        } else {
            ROS_ERROR("Failed to call service");
            return 1;
        }
    } catch (std::exception const& e) {
        ROS_ERROR("Service call failed: %s", e.what());
        return 1;
    }

    return 0;
}