#include "../motor_library/motors_manager.hpp"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h> // To publish heartbeats

void moveDrive(const geometry_msgs::Twist::ConstPtr& msg);
void heartbeatCallback(const ros::TimerEvent&);

MotorsManager driveManager;
std::vector<std::string> driveNames = 
    {"FrontLeft", "FrontRight", "MiddleLeft", "MiddleRight", "BackLeft", "BackRight"};

std::unordered_map<std::string, float> motorMultipliers; // Store the multipliers for each motor

std::optional<float> WHEEL_DISTANCE_INNER;
std::optional<float> WHEEL_DISTANCE_OUTER;
std::optional<float> WHEELS_M_S_TO_MOTOR_REV_S;
std::optional<float> MAX_MOTOR_SPEED_REV_S;

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "drive_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    XmlRpc::XmlRpcValue controllersRoot;
    assert(nh.getParam("motors/controllers", controllersRoot));
    assert(controllersRoot.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    driveManager = MotorsManager(driveNames, controllersRoot);

    // Load motor multipliers from the ROS parameter server
    XmlRpc::XmlRpcValue driveControllers;
    assert(nh.getParam("drive/controllers", driveControllers));
    assert(driveControllers.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (const auto& driveName : driveNames) {
        assert(driveControllers.hasMember(driveName));
        assert(driveControllers[driveName].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        if (driveControllers[driveName].hasMember("multiplier")) {
            motorMultipliers[driveName] = static_cast<double>(driveControllers[driveName]["multiplier"]);
        }
    }

    // Load rover dimensions and other parameters from the ROS parameter server
    float roverWidth;
    float roverLength;
    assert(nh.getParam("rover/width", roverWidth));
    assert(roverWidth.getType() == XmlRpc::XmlRpcValue::TypeDouble);
    assert(nh.getParam("rover/length", roverLength))
    assert(roverLength.getType() == XmlRpc::XmlRpcValue::TypeDouble);
    WHEEL_DISTANCE_INNER = roverWidth / 2.0
    WHEEL_DISTANCE_OUTER = sqrt(((roverWidth / 2.0) ** 2) + ((roverLength / 2.0) ** 2))

    float ratioMotorToWheel;
    assert(nh.getParam("wheel/gear_ratio", ratioMotorToWheel));
    assert(ratioMotorToWheel.getType() == XmlRpc::XmlRpcValue::TypeDouble);
    // To convert m/s to rev/s, multiply by this constant. Divide by circumference, multiply by gear ratio.
    float wheelRadius;
    nh.getParam("wheel/radius", wheelRadius);
    WHEELS_M_S_TO_MOTOR_REV_S = (1 / (wheelRadius * 2 * M_PI)) * ratioMotorToWheel;

    float maxSpeedMPerS = rospy.get_param("rover/max_speed")
    assert(nh.getParam("rover/max_speed", maxSpeedMPerS));
    assert(maxSpeedMPerS.getType() == XmlRpc::XmlRpcValue::TypeDouble);
    assert(maxSpeedMPerS > 0);

    MAX_MOTOR_SPEED_REV_S = maxSpeedMPerS * self.WHEELS_M_S_TO_MOTOR_REV_S;

    // Subscribe to the ROS topic for drive commands
    ros::Subscriber moveDriveSubscriber = n->subscribe<geometry_msgs::Twist>("cmd_vel", 1, moveDrive);

    // Create a 0.1 second heartbeat timer
    ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(0.1), heartbeatCallback);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void moveDrive(const geometry_msgs::Twist::ConstPtr& msg) {
    // Process drive commands and set motor speeds
    float forward = msg->linear.x;
    float turn = msg->angular.z;

    // Calculate motor speeds and adjust for maximum speed
    float turn_difference_inner = turn * WHEEL_DISTANCE_INNER;
    float turn_difference_outer = turn * WHEEL_DISTANCE_OUTER;

    float left_rev_inner = (forward - turn_difference_inner) * WHEELS_M_S_TO_MOTOR_REV_S;
    float right_rev_inner = (forward + turn_difference_inner) * WHEELS_M_S_TO_MOTOR_REV_S;
    float left_rev_outer = (forward - turn_difference_outer) * WHEELS_M_S_TO_MOTOR_REV_S;
    float right_rev_outer = (forward + turn_difference_outer) * WHEELS_M_S_TO_MOTOR_REV_S;

    // If speed too fast, scale to max speed. Ignore inner for comparison since outer > inner, always.
    float larger_abs_rev_s = max(abs(left_rev_outer), abs(right_rev_outer));
    if (larger_abs_rev_s > MAX_MOTOR_SPEED_REV_S) {
        float change_ratio = MAX_MOTOR_SPEED_REV_S / larger_abs_rev_s;
        left_rev_inner *= change_ratio;
        right_rev_inner *= change_ratio;
        left_rev_outer *= change_ratio;
        right_rev_outer *= change_ratio;
    }

    std::unordered_map<std::string, float> driveCommandVelocities = {
        {"FrontLeft", left_rev_outer},
        {"FrontRight", right_rev_outer},
        {"MiddleLeft", left_rev_inner},
        {"MiddleRight", right_rev_inner},
        {"BackLeft", left_rev_outer},
        {"BackRight", right_rev_outer}
    };

    for (const auto& pair : driveCommandVelocities) {
        // Apply the multiplier for each motor
        const std::string& name = pair.first;

        // Set the desired speed for the motor
        float multiplier = motorMultipliers[name];
        float velocity = pair.second * multiplier;  // currently in rad/s

        Controller& controller = driveManager.get_controller(name);
        float vel_rad_s = velocity * 2 * M_PI;
        controller.set_desired_speed_rad_s(vel_rad_s);
    }

    // Set the messageReceived flag to true when a message is received
    messageReceived = true;
}

void heartbeatCallback(const ros::TimerEvent&) {
    // If no message has been received within the last 0.1 seconds, set desired speed to 0 for all motors
    if (!messageReceived) {
        for (const auto& name : driveNames) {
            Controller& controller = driveManager.get_controller(name);
            controller.set_desired_speed_unit(0.0);
        }
    }

    // Reset the messageReceived flag
    messageReceived = false;
}
