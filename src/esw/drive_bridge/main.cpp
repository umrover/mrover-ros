#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <can_manager.hpp>
#include <motors_manager.hpp>

#include <mrover/ControllerState.h>

using namespace mrover;

void moveDrive(const geometry_msgs::Twist::ConstPtr& msg);
void jointDataCallback(const ros::TimerEvent&);
void controllerDataCallback(const ros::TimerEvent&);

std::unique_ptr<MotorsManager> driveManager;
std::vector<std::string> driveNames{"front_left", "front_right", "middle_left", "middle_right", "back_left", "back_right"};

ros::Publisher jointDataPublisher;
ros::Publisher controllerDataPublisher;
std::unordered_map<std::string, Dimensionless> motorMultipliers; // Store the multipliers for each motor

Meters WHEEL_DISTANCE_INNER;
Meters WHEEL_DISTANCE_OUTER;
compound_unit<Radians, inverse<Meters>> WHEEL_LINEAR_TO_ANGULAR;
RadiansPerSecond MAX_MOTOR_SPEED;

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
        if (driveControllers[driveName].hasMember("multiplier") && driveControllers[driveName]["multiplier"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            motorMultipliers[driveName] = Dimensionless{static_cast<double>(driveControllers[driveName]["multiplier"])};
        }
    }

    // Load rover dimensions and other parameters from the ROS parameter server
    auto roverWidth = requireParamAsUnit<Meters>(nh, "rover/width");
    auto roverLength = requireParamAsUnit<Meters>(nh, "rover/length");
    WHEEL_DISTANCE_INNER = roverWidth / 2;
    WHEEL_DISTANCE_OUTER = sqrt(((roverWidth / 2) * (roverWidth / 2)) + ((roverLength / 2) * (roverLength / 2)));

    auto ratioMotorToWheel = requireParamAsUnit<Dimensionless>(nh, "wheel/gear_ratio");
    auto wheelRadius = requireParamAsUnit<Meters>(nh, "wheel/radius");
    // TODO(quintin) is dividing by ratioMotorToWheel right?
    WHEEL_LINEAR_TO_ANGULAR = Radians{1} / wheelRadius * ratioMotorToWheel;

    auto maxLinearSpeed = requireParamAsUnit<MetersPerSecond>(nh, "rover/max_speed");
    assert(maxLinearSpeed > 0_mps);

    MAX_MOTOR_SPEED = maxLinearSpeed * WHEEL_LINEAR_TO_ANGULAR;

    jointDataPublisher = nh.advertise<sensor_msgs::JointState>("drive_joint_data", 1);
    controllerDataPublisher = nh.advertise<ControllerState>("drive_controller_data", 1);

    // Subscribe to the ROS topic for drive commands
    ros::Subscriber moveDriveSubscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, moveDrive);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void moveDrive(const geometry_msgs::Twist::ConstPtr& msg) {
    // See 11.5.1 in "Controls Engineering in the FIRST Robotics Competition" for the math
    auto forward = MetersPerSecond{msg->linear.x};
    auto turn = RadiansPerSecond{msg->angular.z};
    // TODO(quintin)    Don't ask me to explain perfectly why we need to cancel out a meters unit in the numerator
    //                  I think it comes from the fact that there is a unit vector in the denominator of the equation
    auto delta = turn * WHEEL_DISTANCE_INNER / Meters{1};
    RadiansPerSecond left = forward * WHEEL_LINEAR_TO_ANGULAR - delta;
    RadiansPerSecond right = forward * WHEEL_LINEAR_TO_ANGULAR + delta;

    std::unordered_map<std::string, RadiansPerSecond> driveCommandVelocities{
            {"FrontLeft", left},
            {"FrontRight", right},
            {"MiddleLeft", left},
            {"MiddleRight", right},
            {"BackLeft", left},
            {"BackRight", right},
    };

    for (auto [name, angularVelocity]: driveCommandVelocities) {
        // Set the desired speed for the motor
        Dimensionless multiplier = motorMultipliers[name];
        RadiansPerSecond scaledAngularVelocity = angularVelocity * multiplier; // currently in rad/s

        driveManager->get_controller(name).set_desired_velocity(scaledAngularVelocity);
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
    ControllerState controllerData;
    controllerDataPublisher.publish(controllerData);
}
