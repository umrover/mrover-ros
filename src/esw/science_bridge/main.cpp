#include "can_device.hpp"
#include "messaging.hpp"
#include "std_srvs/SetBool.h"
#include <memory>
#include <mrover/CAN.h>
#include <mrover/HeaterData.h>
#include <mrover/ScienceThermistors.h>
#include <mrover/SpectralGroup.h>
#include <ros/ros.h>
#include <unordered_map>

std::unique_ptr<mrover::CanDevice> scienceCanDevice;

void processCANData(const mrover::CAN::ConstPtr& msg);

std::unique_ptr<ros::Publisher> heaterDataPublisher;
std::unique_ptr<ros::Publisher> spectralDataPublisher;
std::unique_ptr<ros::Publisher> thermistorDataPublisher;

bool changeHeaterAutoShutoffState(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    scienceCanDevice->publish_message(mrover::InBoundScienceMessage{mrover::HeaterAutoShutOffCommand{.enable_auto_shutoff = static_cast<bool>(req.data)}});
    res.success = true;
    res.message = "DONE";
    return true;
}

bool enableScienceDeviceCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res, mrover::ScienceDevice scienceDevice) {
    scienceCanDevice->publish_message(mrover::InBoundScienceMessage{mrover::EnableScienceDeviceCommand{.science_device = scienceDevice, .enable = static_cast<bool>(req.data)}});
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

    std::unordered_map<std::string, mrover::ScienceDevice> scienceDeviceByName = {
            {"heater_b0", mrover::ScienceDevice::HEATER_B0},
            {"heater_n0", mrover::ScienceDevice::HEATER_N0},
            {"heater_b1", mrover::ScienceDevice::HEATER_B1},
            {"heater_n1", mrover::ScienceDevice::HEATER_N1},
            {"heater_b2", mrover::ScienceDevice::HEATER_B2},
            {"heater_n2", mrover::ScienceDevice::HEATER_N2},
            {"white_led_0", mrover::ScienceDevice::WHITE_LED_0},
            {"white_led_1", mrover::ScienceDevice::WHITE_LED_1},
            {"white_led_2", mrover::ScienceDevice::WHITE_LED_2},
            {"uv_led_0", mrover::ScienceDevice::UV_LED_0},
            {"uv_led_1", mrover::ScienceDevice::UV_LED_1},
            {"uv_led_2", mrover::ScienceDevice::UV_LED_2},
    };

    scienceCanDevice = std::make_unique<mrover::CanDevice>(nh, "jetson", "science");

    for (auto const& [deviceName, scienceDevice]: scienceDeviceByName) {
        // Advertise services and set the callback using a la0mbda function
        nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>(
                "science_enable_" + deviceName,
                [scienceDevice](std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
                    return enableScienceDeviceCallback(req, res, scienceDevice);
                });
    }

    nh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("science_change_heater_auto_shutoff_state", changeHeaterAutoShutoffState);

    heaterDataPublisher = std::make_unique<ros::Publisher>(nh.advertise<mrover::HeaterData>("science_heater_state", 1));
    spectralDataPublisher = std::make_unique<ros::Publisher>(nh.advertise<mrover::SpectralGroup>("science_spectral", 1));
    thermistorDataPublisher = std::make_unique<ros::Publisher>(nh.advertise<mrover::ScienceThermistors>("scienceThermistors", 1));

    ros::Subscriber CANSubscriber = nh.subscribe<mrover::CAN>("can/pdlb/in", 1, processCANData);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}