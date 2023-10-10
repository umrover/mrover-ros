#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <motors_manager.hpp>

#include <mrover/ControllerState.h>
#include "can_manager.hpp"

void moveDrive(const geometry_msgs::Twist::ConstPtr& msg);
void jointDataCallback(const ros::TimerEvent&);
void controllerDataCallback(const ros::TimerEvent&);

std::unique_ptr<MotorsManager> driveManager;
std::vector<std::string> driveNames{"front_left", "front_right", "middle_left", "middle_right", "back_left", "back_right"};

ros::Publisher jointDataPublisher;
ros::Publisher controllerDataPublisher;
std::unordered_map<std::string, float> motorMultipliers; // Store the multipliers for each motor

float WHEEL_DISTANCE_INNER;
float WHEEL_DISTANCE_OUTER;
float WHEELS_M_S_TO_MOTOR_REV_S;
float MAX_MOTOR_SPEED_REV_S;

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "drive_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    driveManager = std::make_unique<MotorsManager>(nh, "drive", driveNames);

    // Load motor multipliers from the ROS parameter server
    XmlRpc::XmlRpcValue driveControllers;
    assert(nh.hasParam("drive/controllers"));
    nh.getParam("drive/controllers", driveControllers);
    assert(driveControllers.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (const auto& driveName: driveNames) {
        assert(driveControllers.hasMember(driveName));
        assert(driveControllers[driveName].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        if (driveControllers[driveName].hasMember("multiplier") &&  driveControllers[driveName]["multiplier"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            motorMultipliers[driveName] = (float) static_cast<double>(driveControllers[driveName]["multiplier"]);
        }
    }

    // Load rover dimensions and other parameters from the ROS parameter server
    float roverWidth = 0.0f;
    float roverLength = 0.0f;
    assert(nh.hasParam("rover/width"));
    nh.getParam("rover/width", roverWidth);
    assert(nh.hasParam("rover/length"));
    nh.getParam("rover/length", roverLength);
    WHEEL_DISTANCE_INNER = roverWidth / 2.0f;
    WHEEL_DISTANCE_OUTER = std::sqrt(((roverWidth / 2.0f) * (roverWidth / 2.0f)) + ((roverLength / 2.0f) * (roverLength / 2.0f)));

    float ratioMotorToWheel = 0.0f;
    assert(nh.hasParam("wheel/gear_ratio"));
    nh.getParam("wheel/gear_ratio", ratioMotorToWheel);
    // To convert m/s to rev/s, multiply by this constant. Divide by circumference, multiply by gear ratio.
    float wheelRadius = 0.0f;
    nh.getParam("wheel/radius", wheelRadius);
    WHEELS_M_S_TO_MOTOR_REV_S = (1.0f / (wheelRadius * 2.0f * static_cast<float>(std::numbers::pi))) * ratioMotorToWheel;

    float maxSpeedMPerS = 0.0f;
    assert(nh.hasParam("rover/max_speed"));
    nh.getParam("rover/max_speed", maxSpeedMPerS);
    assert(maxSpeedMPerS > 0.0f);

    MAX_MOTOR_SPEED_REV_S = maxSpeedMPerS * WHEELS_M_S_TO_MOTOR_REV_S;

    jointDataPublisher = nh.advertise<sensor_msgs::JointState>("drive_joint_data", 1);
    controllerDataPublisher = nh.advertise<mrover::ControllerState>("drive_controller_data", 1);

    // Subscribe to the ROS topic for drive commands
    ros::Subscriber moveDriveSubscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, moveDrive);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void moveDrive(const geometry_msgs::Twist::ConstPtr& msg) {
    // Process drive commands and set motor speeds
    auto forward = static_cast<float>(msg->linear.x);
    auto turn = static_cast<float>(msg->angular.z);

    // Calculate motor speeds and adjust for maximum speed
    float turn_difference_inner = turn * WHEEL_DISTANCE_INNER;
    float turn_difference_outer = turn * WHEEL_DISTANCE_OUTER;

    float left_rev_inner = (forward - turn_difference_inner) * WHEELS_M_S_TO_MOTOR_REV_S;
    float right_rev_inner = (forward + turn_difference_inner) * WHEELS_M_S_TO_MOTOR_REV_S;
    float left_rev_outer = (forward - turn_difference_outer) * WHEELS_M_S_TO_MOTOR_REV_S;
    float right_rev_outer = (forward + turn_difference_outer) * WHEELS_M_S_TO_MOTOR_REV_S;

    // If speed too fast, scale to max speed. Ignore inner for comparison since outer > inner, always.
    float larger_abs_rev_s = fmax(abs(left_rev_outer), abs(right_rev_outer));
    if (larger_abs_rev_s > MAX_MOTOR_SPEED_REV_S) {
        float change_ratio = MAX_MOTOR_SPEED_REV_S / larger_abs_rev_s;
        left_rev_inner *= change_ratio;
        right_rev_inner *= change_ratio;
        left_rev_outer *= change_ratio;
        right_rev_outer *= change_ratio;
    }

    std::unordered_map<std::string, float> driveCommandVelocities{
            {"FrontLeft", left_rev_outer},
            {"FrontRight", right_rev_outer},
            {"MiddleLeft", left_rev_inner},
            {"MiddleRight", right_rev_inner},
            {"BackLeft", left_rev_outer},
            {"BackRight", right_rev_outer},
    };

    for (const auto& pair: driveCommandVelocities) {
        // Apply the multiplier for each motor
        const std::string& name = pair.first;

        // Set the desired speed for the motor
        float multiplier = motorMultipliers[name];
        float velocity = pair.second * multiplier; // currently in rad/s

        Controller& controller = driveManager->get_controller(name);
        float vel_rad_s = velocity * 2.0f * static_cast<float>(std::numbers::pi);
        controller.set_desired_velocity(vel_rad_s);
    }

    driveManager->updateLastConnection();
}

void jointDataCallback(const ros::TimerEvent&) {
    //TODO
    sensor_msgs::JointState jointData; // TODO
    jointDataPublisher.publish(jointData);
}

void controllerDataCallback(const ros::TimerEvent&) {
    //TODO
    mrover::ControllerState controllerData;
    controllerDataPublisher.publish(controllerData);
}
