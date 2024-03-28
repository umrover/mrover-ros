#include "messaging.hpp"
#include <mrover/CAN.h>
#include <mrover/PDLB.h>
#include <ros/ros.h>

void processCANData(const mrover::CAN::ConstPtr& msg);

ros::Publisher PDBPublisher;

auto main(int argc, char** argv) -> int {
    // Initialize the ROS node
    ros::init(argc, argv, "pdb_bridge");
    ros::NodeHandle nh;

    PDBPublisher = nh.advertise<mrover::PDLB>("pdlb_data", 1);
    ros::Subscriber CANSubscriber = nh.subscribe<mrover::CAN>("can/pdlb/in", 1, processCANData);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void processMessage(mrover::PDBData const& message) {
    mrover::PDLB pdlb_data;
    for (int i = 0; i < 6; ++i) {
        pdlb_data.temperatures.at(i) = message.temperatures[i];
        pdlb_data.currents.at(i) = message.currents[i];
    }
    PDBPublisher.publish(pdlb_data);
}

void processCANData(const mrover::CAN::ConstPtr& msg) {

    assert(msg->source == "pdlb");
    assert(msg->destination == "jetson");

    mrover::OutBoundPDLBMessage const& message = *reinterpret_cast<mrover::OutBoundPDLBMessage const*>(msg->data.data());

    // This calls the correct process function based on the current value of the alternative
    std::visit([&](auto const& messageAlternative) { processMessage(messageAlternative); }, message);
}
