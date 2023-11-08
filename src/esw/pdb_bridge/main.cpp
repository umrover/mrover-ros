#include "messaging.hpp"
#include <mrover/CAN.h>
#include <mrover/PDLB.h>
#include <ros/ros.h>

void processCANData(const mrover::CAN::ConstPtr& msg);

ros::Publisher PDBPublisher;

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "pdb_bridge");
    ros::NodeHandle nh;

    PDBPublisher = nh.advertise<mrover::PDLB>("can/pdlb/out", 1);
    ros::Subscriber CANSubscriber = nh.subscribe<mrover::CAN>("can/pdlb/in", 1, processCANData);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void processMessage(PDBData const& message) {
    mrover::PDLB pdlb_data;
    pdlb_data.temperatures = {
            message.temperature_24v,
            message.temperature_12v_jetson,
            message.temperature_12v_rest,
            message.temperature_12v_buck,
            message.temperature_5v,
            message.temperature_3v3};
    pdlb_data.currents = {
            message.current_24v,
            message.current_12v_jetson,
            message.temperature_12v_rest,
            message.current_12v_buck,
            message.current_5v,
            message.current_3v3};
    PDBPublisher.publish(pdlb_data);
}

void processCANData(const mrover::CAN::ConstPtr& msg) {

    // TODO - there is a lot of code that needs to be fixed here
    assert(msg->source == "pdlb");
    assert(msg->destination == "jetson");


    OutBoundPDLBMessage const& message = *reinterpret_cast<OutBoundPDLBMessage const*>(msg->data.data());

    // This calls the correct process function based on the current value of the alternative
    std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
}
