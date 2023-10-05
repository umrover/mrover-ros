#include <ros/ros.h>
#include <mrover/PDB.h>
#include <mrover/CAN.h>

void processCANData(const mrover::CAN::ConstPtr& msg);

ros::Publisher PDBPublisher;

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "pdb_bridge");
    ros::NodeHandle nh;

    PDBPublisher = nh.advertise<mrover::PDB>("pdb_data", 1);
    ros::Subscriber CANSubscriber = nh.subscribe<mrover::CAN>("can_data", 1, processCANData);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void processCANData(const mrover::CAN::ConstPtr& msg) {
    if (msg->bus == 0 && msg->message_id == 0) {
        return;
        // TODO
    }

    mrover::PDB PDBData;
    PDBData.temperatures = {0, 0, 0, 0, 0};  // TODO
    PDBData.currents = {0, 0, 0, 0, 0};  // TODO
    PDBPublisher.publish(PDBData);
}