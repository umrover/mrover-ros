#include "../motor_library/motors_manager.hpp"
#include <ros/ros.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>
#include <mrover/Position.h>
#include <std_msgs/Float32.h> // To publish heartbeats

void moveMastGimbalThrottle(const mrover::Throttle::ConstPtr& msg);
void moveMastGimbalVelocity(const mrover::Velocity::ConstPtr& msg);
void moveMastGimbalPosition(const mrover::Position::ConstPtr& msg);
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
    ros::Subscriber moveMastGimbalThrottleSubscriber = n->subscribe<mrover::Throttle>("mast_gimbal_throttle_cmd", 1, moveMastGimbalThrottle);
    ros::Subscriber moveMastGimbalVelocityeSubscriber = n->subscribe<mrover::Velocity>("mast_gimbal_velocity_cmd", 1, moveMastGimbalVelocity);
    ros::Subscriber moveMastGimbalPositionSubscriber = n->subscribe<mrover::Position>("mast_gimbal_position_cmd", 1, moveMastGimbalPosition);

    // Create a 0.1 second heartbeat timer
    ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(0.1), heartbeatCallback);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void moveMastGimbalThrottle(const mrover::Throttle::ConstPtr& msg) {
    if (msg->names != mastGimbalNames && msg->names.size() != msg->throttle.size()) {
        ROS_ERROR("Mast Gimbal request is invalid!");
        return;
    }
    for (size_t i = 0; i < msg->names.size(); ++i) {
        std::string& name = msg->names[i];
        Controller& controller = *mastGimbalManager.get_controller(name);
        float throttle = std::clamp(msg->throttle[i], -1.0, 1.0);
        controller.set_desired_throttle(throttle);
    }

    // Set the messageReceived flag to true when a message is received
    messageReceived = true;
}

void moveMastGimbalVelocity(const mrover::Velocity::ConstPtr& msg) {
    if (msg->names != mastGimbalNames && msg->names.size() != msg->velocity.size()) {
        ROS_ERROR("Mast Gimbal request is invalid!");
        return;
    }
    for (size_t i = 0; i < msg->names.size(); ++i) {
        std::string& name = msg->names[i];
        Controller& controller = *mastGimbalManager.get_controller(name);
        float velocity = std::clamp(msg->velocity[i], -1.0, 1.0);  // TODO
        controller.set_desired_throttle(velocity);
    }

    // Set the messageReceived flag to true when a message is received
    messageReceived = true;
}

void moveMastGimbalPositionSubscriber(const mrover::Position::ConstPtr& msg) {
    if (msg->names != mastGimbalNames && msg->names.size() != msg->position.size()) {
        ROS_ERROR("Mast Gimbal request is invalid!");
        return;
    }
    for (size_t i = 0; i < msg->names.size(); ++i) {
        std::string& name = msg->names[i];
        Controller& controller = *mastGimbalManager.get_controller(name);
        float position = 0.0;

        // TODO - change the position and make sure to clamp it

        controller.set_desired_position(position);
    }

    // Set the messageReceived flag to true when a message is received
    messageReceived = true;
}

void heartbeatCallback(const ros::TimerEvent&) {
    // If no message has been received within the last 0.1 seconds, set desired speed to 0 for all motors
    if (!messageReceived) {
        for (const auto& mastGimbalName : mastGimbalNames) {
            Controller& controller = *mastGimbalManager.get_controller(mastGimbalName);
            controller.set_desired_throttle(0.0);
        }
    }

    // Reset the messageReceived flag
    messageReceived = false;
}
