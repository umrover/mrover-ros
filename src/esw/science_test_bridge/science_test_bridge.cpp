#include "messaging.hpp"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <mrover/HeaterData.h>
#include <mrover/ScienceThermistors.h>
#include <mrover/SpectralGroup.h>


// void printHeater(mrover::HeaterData::ConstPtr &msg) {
//     ROS_INFO("heater 0: %d  heater 1: %d    heater 2: %d    heater 3: %d    heater 4: %d    heater 5: %d", 
//         msg->state.at(0), msg->state.at(1), msg->state.at(2), msg->state.at(3), msg->state.at(4), msg->state.at(5));
// }

// void printThermistor(mrover::ScienceThermistors::ConstPtr &msg) {
//     ROS_INFO("temp 0: %f  temp 1: %f    temp 2: %f    temp 3: %f    temp 4: %f    temp 5: %f", 
//         msg->temps.at(0).temperature, msg->temps.at(1).temperature, msg->temps.at(2).temperature, msg->temps.at(3).temperature, msg->temps.at(4).temperature, msg->temps.at(5).temperature);
// }

// void printSpectral(mrover::SpectralGroup &msg) {
//     ROS_INFO("spectral 0: %f, spectral 1: %f, spectral 2: %f",
//     msg.data.at(0), msg.spectrals.at(1), msg.spectrals.at(2));
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "science_test_bridge");
    ros::NodeHandle nh;

    // read heater data
    // ros::Subscriber heaterSubscriber = nh.subscribe<mrover::HeaterData>("science_heater_state", 1, printHeater);
    // // read thermistor data
    // ros::Subscriber thermSubscriber = nh.subscribe<mrover::ScienceThermistors>("science_thermistors", 1, printThermistor);
    // read spectral data
    // ros::Subscriber spectralSubscriber = nh.subscribe<mrover::SpectralGroup>("science_spectral", 1);

    // turn on LED
    

    // ros::service::waitForService("/science_change_heater_auto_shutoff_state");
    while (ros::ok()){
        try {
            ros::ServiceClient service_client = nh.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("/science_enable_white_led_0");

            std_srvs::SetBool::Request request;
            request.data = true;


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
    }

    return 0;
}