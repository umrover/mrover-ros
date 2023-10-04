#include <ros/ros.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "science_bridge");
    ros::NodeHandle nh;

    // Enter the ROS event loop
    ros::spin();

    return 0;
}