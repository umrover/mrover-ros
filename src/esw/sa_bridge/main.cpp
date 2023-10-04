#include <ros/ros.h>

#include <motors_manager.hpp>

std::unique_ptr<MotorsManager> SAManager;
std::vector<std::string> SANames =
        {"sa_x", "sa_y", "sa_z", "scoop", "drill"};

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "sa_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    SAManager = std::make_unique<MotorsManager>(nh, "sa", SANames);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}