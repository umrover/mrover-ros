#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <can_device.hpp>
#include <motors_group.hpp>
#include <params_utils.hpp>
#include <units/units.hpp>

/*
 *  Converts from a Twist (linear and angular velocity) to the individual wheel velocities
 */

using namespace mrover;

void moveDrive(geometry_msgs::Twist::ConstPtr const& msg);

ros::Publisher leftVelocityPub, rightVelocityPub;

Meters WHEEL_DISTANCE_INNER;
compound_unit<Radians, inverse<Meters>> WHEEL_LINEAR_TO_ANGULAR;

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "drive_bridge");
    ros::NodeHandle nh;

    auto roverWidth = requireParamAsUnit<Meters>(nh, "rover/width");
    WHEEL_DISTANCE_INNER = roverWidth / 2;

    auto ratioMotorToWheel = requireParamAsUnit<Dimensionless>(nh, "wheel/gear_ratio");
    auto wheelRadius = requireParamAsUnit<Meters>(nh, "wheel/radius");
    WHEEL_LINEAR_TO_ANGULAR = Radians{1} / wheelRadius * ratioMotorToWheel;

    auto leftGroup = std::make_unique<MotorsGroup>(nh, "drive_left");
    auto rightGroup = std::make_unique<MotorsGroup>(nh, "drive_right");

    leftVelocityPub = nh.advertise<Velocity>("drive_left_velocity_cmd", 1);
    rightVelocityPub = nh.advertise<Velocity>("drive_right_velocity_cmd", 1);

    auto twistSubscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, moveDrive);

    ros::spin();

    return 0;
}

void moveDrive(geometry_msgs::Twist::ConstPtr const& msg) {
    // See 13.3.1.4 in "Modern Robotics" for the math
    // Link: https://hades.mech.northwestern.edu/images/7/7f/MR.pdf 
    auto forward = MetersPerSecond{msg->linear.x};
    auto turn = RadiansPerSecond{msg->angular.z};
    // TODO(quintin)    Don't ask me to explain perfectly why we need to cancel out a meters unit in the numerator
    //                  I think it comes from the fact that there is a unit vector in the denominator of the equation
    auto delta = turn / Radians{1} * WHEEL_DISTANCE_INNER; // should be in m/s
    RadiansPerSecond left = (forward - delta) * WHEEL_LINEAR_TO_ANGULAR;
    RadiansPerSecond right = (forward + delta) * WHEEL_LINEAR_TO_ANGULAR;
    RadiansPerSecond maximal = std::max(abs(right), abs(left));
    if (maximal > RadiansPerSecond{10 * 2 * M_PI}) {
        Dimensionless changeRatio = RadiansPerSecond{10 * 2 * M_PI} / maximal;
        left = left * changeRatio;
        right = right * changeRatio;
    }
    {
        Velocity leftVelocity;
        leftVelocity.names = {"front_left", "middle_left", "back_left"};
        leftVelocity.velocities.resize(leftVelocity.names.size(), left.get());
        leftVelocity.velocities.resize(leftVelocity.names.size(), left.get());
        leftVelocityPub.publish(leftVelocity);
    }
    {
        Velocity rightVelocity;
        rightVelocity.names = {"front_right", "middle_right", "back_right"};
        rightVelocity.velocities.resize(rightVelocity.names.size(), right.get());
        rightVelocityPub.publish(rightVelocity);
    }
}
