#include "../motor_library/motors_manager.hpp"
#include <ros/ros.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>
#include <mrover/Position.h>
#include <std_msgs/Float32.h> // To publish heartbeats
#include <algorithm>
#include <chrono>

void moveArmThrottle(const mrover::Throttle::ConstPtr& msg);
void moveArmVelocity(const mrover::Velocity::ConstPtr& msg); 
void moveArmPosition(const mrover::Position::ConstPtr& msg); 
void heartbeatCallback(const ros::TimerEvent&);

std::optional<MotorsManager> armManager;
std::vector<std::string> armNames =
        {"joint_a", "joint_b", "joint_c", "joint_de", "finger", "gripper"};

std::chrono::high_resolution_clock::time_point lastConnection = std::chrono::high_resolution_clock::now();


std::unordered_map<std::string, float> motorMultipliers; // Store the multipliers for each motor

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "arm_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    XmlRpc::XmlRpcValue controllersRoot;
    assert(nh.getParam("motors/controllers", controllersRoot));
    assert(controllersRoot.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    armManager = MotorsManager(&nh, armNames, controllersRoot);

    // Load motor multipliers from the ROS parameter server
    XmlRpc::XmlRpcValue armControllers;
    assert(nh.getParam("arm/controllers", armControllers));
    assert(armControllers.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (const auto& armName: armNames) {
        assert(armControllers.hasMember(armName));
        assert(armControllers[armName].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        if (armControllers[armName].hasMember("multiplier")) {
            motorMultipliers[armName] = static_cast<double>(armControllers[armName]["multiplier"]);
        }
    }

    // Subscribe to the ROS topic for arm commands
    ros::Subscriber moveArmThrottleSub = nh.subscribe<mrover::Throttle>("ra_throttle_cmd", 1, moveArmThrottle);
    ros::Subscriber moveArmVelocitySub = nh.subscribe<mrover::Velocity>("ra_velocity_cmd", 1, moveArmVelocity);
    ros::Subscriber moveArmPositionSub = nh.subscribe<mrover::Position>("ra_position_cmd", 1, moveArmPosition);

    // Create a 0.1 second heartbeat timer
    ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(0.1), heartbeatCallback);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void moveArmThrottle(const mrover::Throttle::ConstPtr& msg) {
    if (msg->names != armNames && msg->names.size() != msg->throttles.size()) {
        ROS_ERROR("Arm request is invalid!");
        return;
    }

    lastConnection = std::chrono::high_resolution_clock::now();
    
    for (size_t i = 0; i < msg->names.size(); ++i) {
        const std::string& name = msg->names[i];
        Controller& controller = *armManager.value().get_controller(name);
        float throttle = std::clamp(msg->throttles[i], -1.0f, 1.0f);
        controller.set_desired_throttle(throttle);
    }
}

void moveArmVelocity(const mrover::Velocity::ConstPtr& msg) {
    if (msg->names != armNames && msg->names.size() != msg->velocities.size()) {
        ROS_ERROR("Arm request is invalid!");
        return;
    }

    lastConnection = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < msg->names.size(); ++i) {
        const std::string& name = msg->names[i];
        Controller& controller = *armManager.value().get_controller(name);
        float velocity = std::clamp(msg->velocities[i], -1.0f, 1.0f);

        // TODO - need to take into consideration gear ratio

        controller.set_desired_velocity(velocity);
    }
}

void moveArmPosition(const mrover::Position::ConstPtr& msg) {
    if (msg->names != armNames && msg->names.size() != msg->positions.size()) {
        ROS_ERROR("Arm request is invalid!");
        return;
    }

    lastConnection = std::chrono::high_resolution_clock::now();

    for (size_t i = 0; i < msg->names.size(); ++i) {
        const std::string& name = msg->names[i];
        Controller& controller = *armManager.value().get_controller(name);
        float position = 0.0f;

        // TODO - change the position and make sure to clamp it

        controller.set_desired_position(position);
    }
}

void heartbeatCallback(const ros::TimerEvent&) {
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - lastConnection);
    if (duration.count() < 100) {
        for (const auto& armName: armNames) {
            Controller& controller = *armManager.value().get_controller(armName);
            controller.set_desired_throttle(0.0);
        }
    }
}
