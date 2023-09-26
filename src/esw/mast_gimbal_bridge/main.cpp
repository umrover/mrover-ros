#include "../motor_library/motors_manager.hpp"
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h> // To publish heartbeats

void moveMastGimbal(const sensor_msgs::JointState::ConstPtr& msg);
void heartbeatCallback(const ros::TimerEvent&);

MotorsManager mastGimbalManager;
std::vector<std::string> mastGimbalNames = 
    {"mast_gimbal_x", "mast_gimbal_y"};

std::unordered_map<std::string, float> motorMultipliers; // Store the multipliers for each motor

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "mast_gimbal_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    XmlRpc::XmlRpcValue controllersRoot;
    assert(nh.getParam("motors/controllers", controllersRoot));
    assert(controllersRoot.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    mastGimbalManager = MotorsManager(mastGimbalNames, controllersRoot);

    // Subscribe to the ROS topic for arm commands
    ros::Subscriber moveMastGimbalSubscriber = n->subscribe<sensor_msgs::JointState>("mast_gimbal_cmd", 1, moveMastGimbal);

    // Create a 0.1 second heartbeat timer
    ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(0.1), heartbeatCallback);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void moveMastGimbal(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->name != mastGimbalName && msg->name.size() != msg->name.velocity.size()) {
        ROS_ERROR("Mast Gimbal request is invalid!");
        return;
    }
    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::string& name = msg->name[i];
        Controller& controller = mastGimbalManager.get_controller(name);
        float velocity = std::clamp(msg->velocity[i], -1.0, 1.0);
        controller.set_desired_speed_unit(velocity);
    }

    // Set the messageReceived flag to true when a message is received
    messageReceived = true;
}

void heartbeatCallback(const ros::TimerEvent&) {
    // If no message has been received within the last 0.1 seconds, set desired speed to 0 for all motors
    if (!messageReceived) {
        for (const auto& mastGimbalName : mastGimbalNames) {
            Controller& controller = mastGimbalManager.get_controller(mastGimbalName);
            controller.set_desired_speed(0.0);
        }
    }

    // Reset the messageReceived flag
    messageReceived = false;
}
