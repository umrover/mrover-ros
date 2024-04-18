#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <can_device.hpp>
#include <motors_group.hpp>
#include <params_utils.hpp>

using namespace mrover;

void moveDrive(geometry_msgs::Twist::ConstPtr const& msg);

ros::Publisher leftVelocityPub, rightVelocityPub;

Meters WHEEL_DISTANCE_INNER;
compound_unit<Radians, inverse<Meters>> WHEEL_LINEAR_TO_ANGULAR;
RadiansPerSecond MAX_MOTOR_SPEED;

auto main(int argc, char** argv) -> int {
    // Initialize the ROS node
    ros::init(argc, argv, "drive_bridge");
    ros::NodeHandle nh;

    // Load rover dimensions and other parameters from the ROS parameter server
    auto roverWidth = requireParamAsUnit<Meters>(nh, "rover/width");
    WHEEL_DISTANCE_INNER = roverWidth / 2;

    auto ratioMotorToWheel = requireParamAsUnit<Dimensionless>(nh, "wheel/gear_ratio");
    auto wheelRadius = requireParamAsUnit<Meters>(nh, "wheel/radius");
    // TODO(quintin) is dividing by ratioMotorToWheel right?
    WHEEL_LINEAR_TO_ANGULAR = Radians{1} / wheelRadius * ratioMotorToWheel;

    auto maxLinearSpeed = requireParamAsUnit<MetersPerSecond>(nh, "rover/max_speed");
    assert(maxLinearSpeed > 0_mps);

    MAX_MOTOR_SPEED = maxLinearSpeed * WHEEL_LINEAR_TO_ANGULAR;

    auto leftGroup = std::make_unique<MotorsGroup>(nh, "drive_left");
    auto rightGroup = std::make_unique<MotorsGroup>(nh, "drive_right");

    leftVelocityPub = nh.advertise<Velocity>("drive_left_velocity_cmd", 1);
    rightVelocityPub = nh.advertise<Velocity>("drive_right_velocity_cmd", 1);

    // Subscribe to the ROS topic for drive commands
    auto twistSubscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, moveDrive);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void moveDrive(geometry_msgs::Twist::ConstPtr const& msg) {
    // See 11.5.1 in "Controls Engineering in the FIRST Robotics Competition" for the math
    auto forward = MetersPerSecond{msg->linear.x};
    auto turn = RadiansPerSecond{msg->angular.z};
    // TODO(quintin)    Don't ask me to explain perfectly why we need to cancel out a meters unit in the numerator
    //                  I think it comes from the fact that there is a unit vector in the denominator of the equation
    auto delta = turn / Radians{1} * WHEEL_DISTANCE_INNER; // should be in m/s
    RadiansPerSecond left = (forward - delta) * WHEEL_LINEAR_TO_ANGULAR;
    RadiansPerSecond right = (forward + delta) * WHEEL_LINEAR_TO_ANGULAR;

    {
        Velocity leftVelocity;
        leftVelocity.names = {"front_left", "middle_left", "back_left"};
        // TODO(quintin): Invert in firmware
        leftVelocity.velocities = {-left.get(), -left.get(), -left.get()};
        leftVelocityPub.publish(leftVelocity);
    }
    {
        Velocity rightVelocity;
        rightVelocity.names = {"front_right", "middle_right", "back_right"};
        rightVelocity.velocities = {right.get(), right.get(), right.get()};
        rightVelocityPub.publish(rightVelocity);
    }
}
