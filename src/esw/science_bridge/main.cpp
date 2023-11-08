#include "messaging.hpp"
#include "std_srvs/SetBool.h"
#include <memory>
#include <mrover/CAN.h>
#include <mrover/HeaterData.h>
#include <mrover/ScienceThermistors.h>
#include <mrover/SpectralGroup.h>
#include <ros/ros.h>
#include <unordered_map>

void processCANData(const mrover::CAN::ConstPtr& msg);

std::unordered_map<std::string, int> device_name_to_index{
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
        {"uv_led_2", 11},
};

std::unique_ptr<ros::Publisher> heaterDataPublisher;
std::unique_ptr<ros::Publisher> spectralDataPublisher;
std::unique_ptr<ros::Publisher> thermistorDataPublisher;

bool serviceCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res, int device_id) {
    ROS_INFO("TODO - request for device_id %i. Value: %s", device_id, req.data ? "true" : "false");
    res.success = true;
    res.message = "DONE";
    return true;
}

void processMessage(mrover::HeaterStateData const& message) {
    mrover::HeaterData heaterData;
    heaterData.b0 = message.heater_state_info.b0;
    heaterData.n0 = message.heater_state_info.n0;
    heaterData.b1 = message.heater_state_info.b1;
    heaterData.n1 = message.heater_state_info.n1;
    heaterData.b2 = message.heater_state_info.b2;
    heaterData.n2 = message.heater_state_info.n2;

    heaterDataPublisher->publish(heaterData);
}

void processMessage(mrover::SpectralData const& message) {
    mrover::SpectralGroup spectralData;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 6; ++j) {
            spectralData.spectrals[i].data[j] = message.spectrals[i].data[j];
        }
    }
    spectralDataPublisher->publish(spectralData);
}

void processMessage(mrover::ThermistorData const& message) {
    mrover::ScienceThermistors scienceThermistors;
    scienceThermistors.b0.temperature = message.b0;
    scienceThermistors.n0.temperature = message.n0;
    scienceThermistors.b1.temperature = message.b1;
    scienceThermistors.n1.temperature = message.n1;
    scienceThermistors.b2.temperature = message.b2;
    scienceThermistors.n2.temperature = message.n2;
    thermistorDataPublisher->publish(scienceThermistors);
}

void processCANData(const mrover::CAN::ConstPtr& msg) {

    assert(msg->source == "science");
    assert(msg->destination == "jetson");

    mrover::OutBoundScienceMessage const& message = *reinterpret_cast<mrover::OutBoundScienceMessage const*>(msg->data.data());

    // This calls the correct process function based on the current value of the alternative
    std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "science_bridge");
    ros::NodeHandle nh;

    for (auto const& [deviceName, deviceID]: device_name_to_index) {
        // Advertise services and set the callback using a lambda function
        nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
                "science_enable_" + deviceName,
                [deviceID](std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
                    return serviceCallback(req, res, deviceID);
                });
    }

    heaterDataPublisher = std::make_unique<ros::Publisher>(nh.advertise<mrover::HeaterData>("science_heater_state", 1));
    spectralDataPublisher = std::make_unique<ros::Publisher>(nh.advertise<mrover::SpectralGroup>("science_spectral", 1));
    thermistorDataPublisher = std::make_unique<ros::Publisher>(nh.advertise<mrover::ScienceThermistors>("scienceThermistors", 1));

    ros::Subscriber CANSubscriber = nh.subscribe<mrover::CAN>("can/pdlb/in", 1, processCANData);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}