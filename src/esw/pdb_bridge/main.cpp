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

void processCANData(const mrover::CAN::ConstPtr& msg) {
    mrover::PDLB PDBData;
    PDBData.temperatures = {0, 0, 0, 0, 0}; // TODO
    PDBData.currents = {0, 0, 0, 0, 0};     // TODO
    PDBPublisher.publish(PDBData);
}
