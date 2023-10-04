#include <chrono>

#include <ros/ros.h>

#include <motors_manager.hpp>

#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>

void moveMastGimbalThrottle(const mrover::Throttle::ConstPtr& msg);
void moveMastGimbalVelocity(const mrover::Velocity::ConstPtr& msg);
void moveMastGimbalPosition(const mrover::Position::ConstPtr& msg);
void heartbeatCallback(const ros::TimerEvent&);

std::unique_ptr<MotorsManager> mastGimbalManager;
std::vector<std::string> mastGimbalNames =
        {"mast_gimbal_x", "mast_gimbal_y"};


std::chrono::high_resolution_clock::time_point lastConnection = std::chrono::high_resolution_clock::now();
std::unordered_map<std::string, double> motorMultipliers; // Store the multipliers for each motor

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "mast_gimbal_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    mastGimbalManager = std::make_unique<MotorsManager>(nh, mastGimbalNames);

    // Subscribe to the ROS topic for arm commands
    ros::Subscriber moveMastGimbalThrottleSubscriber = nh.subscribe<mrover::Throttle>("mast_gimbal_throttle_cmd", 1, moveMastGimbalThrottle);
    ros::Subscriber moveMastGimbalVelocitySubscriber = nh.subscribe<mrover::Velocity>("mast_gimbal_velocity_cmd", 1, moveMastGimbalVelocity);
    ros::Subscriber moveMastGimbalPositionSubscriber = nh.subscribe<mrover::Position>("mast_gimbal_position_cmd", 1, moveMastGimbalPosition);

    // Create a 0.1 second heartbeat timer
    ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(0.1), heartbeatCallback);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void moveMastGimbalThrottle(const mrover::Throttle::ConstPtr& msg) {
    if (msg->names != mastGimbalNames && msg->names.size() != msg->throttles.size()) {
        ROS_ERROR("Mast Gimbal request is invalid!");
        return;
    }

    lastConnection = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < msg->names.size(); ++i) {
        const std::string& name = msg->names[i];
        Controller& controller = mastGimbalManager->get_controller(name);
        controller.set_desired_throttle(msg->throttles[i]);
    }
}

void moveMastGimbalVelocity(const mrover::Velocity::ConstPtr& msg) {
    if (msg->names != mastGimbalNames && msg->names.size() != msg->velocities.size()) {
        ROS_ERROR("Mast Gimbal request is invalid!");
        return;
    }

    lastConnection = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < msg->names.size(); ++i) {
        const std::string& name = msg->names[i];
        Controller& controller = mastGimbalManager->get_controller(name);
        controller.set_desired_throttle(msg->velocities[i]);
    }
}

void moveMastGimbalPosition(const mrover::Position::ConstPtr& msg) {
    if (msg->names != mastGimbalNames && msg->names.size() != msg->positions.size()) {
        ROS_ERROR("Mast Gimbal request is invalid!");
        return;
    }

    lastConnection = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < msg->names.size(); ++i) {
        const std::string& name = msg->names[i];
        Controller& controller = mastGimbalManager->get_controller(name);
        controller.set_desired_position(msg->positions[i]);
    }
}

void heartbeatCallback(const ros::TimerEvent&) {
    // If no message has been received within the last 0.1 seconds, set desired speed to 0 for all motors
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastConnection);
    if (duration.count() < 100) {
        for (const auto& mastGimbalName: mastGimbalNames) {
            Controller& controller = mastGimbalManager->get_controller(mastGimbalName);
            controller.set_desired_throttle(0.0);
        }
    }
}
