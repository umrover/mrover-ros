#include <ros/init.h>
#include <ros/node_handle.h>

#include <mrover/Position.h>

ros::Subscriber sub;

int main(int argc, char** argv) {
    ros::init(argc, argv, "sim_arm_bridge");
    ros::NodeHandle nh;
    return EXIT_SUCCESS;
}