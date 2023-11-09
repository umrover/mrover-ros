#include "can_device.hpp"
#include <motors_manager.hpp>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

std::vector<std::string> SANames{"sa_x", "sa_y", "sa_z", "scoop", "sensor_actuator"};

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "sa_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    [[maybe_unused]] auto SAManager = std::make_unique<mrover::MotorsManager>(nh, "sa", SANames);
    // Enter the ROS event loop
    ros::spin();

    return 0;
}