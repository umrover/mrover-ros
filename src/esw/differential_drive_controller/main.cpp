#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <can_device.hpp>
#include <params_utils.hpp>
#include <units/units.hpp>

#include <algorithm>
#include <memory>
#include <ranges>
#include <string>

#include <mrover/Velocity.h>

/*
 *  Converts from a Twist (linear and angular velocity) to the individual wheel velocities.
 *  This is for a 6-wheel differential drive rover.
 */

using namespace mrover;

void moveDrive(geometry_msgs::Twist::ConstPtr const& msg);

ros::Publisher leftVelocityPub, rightVelocityPub;

Meters WHEEL_DISTANCE_INNER;
Meters WHEEL_DISTANCE_OUTER;
compound_unit<Radians, inverse<Meters>> WHEEL_LINEAR_TO_ANGULAR;
RevolutionsPerSecond MAX_SPEED;
std::vector<std::string> LEFT_NAMES{"front_left", "middle_left", "back_left"};
std::vector<std::string> RIGHT_NAMES{"front_right", "middle_right", "back_right"};

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "differential_drive_controller");
    ros::NodeHandle nh;

    auto roverWidth = requireParamAsUnit<Meters>(nh, "rover/width");
    auto roverLength = requireParamAsUnit<Meters>(nh, "rover/length");

    {
        std::vector<std::string> allNames;
        allNames.insert(allNames.end(), LEFT_NAMES.begin(), LEFT_NAMES.end());
        allNames.insert(allNames.end(), RIGHT_NAMES.begin(), RIGHT_NAMES.end());
        MAX_SPEED = std::ranges::min(allNames | std::views::transform([&nh](auto name) {
                                         return requireParamAsUnit<RevolutionsPerSecond>(nh, std::format("brushless_motors/controllers/{}/max_velocity", name));
                                     }),
                                     std::less{});
    }

    WHEEL_DISTANCE_INNER = roverWidth / 2;
    WHEEL_DISTANCE_OUTER = sqrt(square(roverWidth / 2) + square(roverLength / 2)); // Pythagorean theorem

    auto wheelRadius = requireParamAsUnit<Meters>(nh, "wheel/radius");
    WHEEL_LINEAR_TO_ANGULAR = Radians{1} / wheelRadius;

    leftVelocityPub = nh.advertise<Velocity>("drive_left_velocity_cmd", 1);
    rightVelocityPub = nh.advertise<Velocity>("drive_right_velocity_cmd", 1);

    auto twistSubscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, moveDrive);

    ros::spin();

    return 0;
}

auto moveDrive(geometry_msgs::Twist::ConstPtr const& msg) -> void {
    // See 13.3.1.4 in "Modern Robotics" for the math
    // Link: https://hades.mech.northwestern.edu/images/7/7f/MR.pdf

    auto forward = MetersPerSecond{msg->linear.x};
    auto turn = RadiansPerSecond{msg->angular.z};

    // TODO(quintin):   Don't ask me to explain perfectly why we need to cancel out a meters unit in the numerator
    //                  I think it comes from the fact that there is a unit vector in the denominator of the equation

    // The outer wheel needs to cover more ground than the inner wheel, so spin at a higher angular velocity

    auto deltaInner = turn / Radians{1} * WHEEL_DISTANCE_INNER;
    auto deltaOuter = turn / Radians{1} * WHEEL_DISTANCE_OUTER;
    RadiansPerSecond leftInner = (forward - deltaInner) * WHEEL_LINEAR_TO_ANGULAR;
    RadiansPerSecond rightInner = (forward + deltaInner) * WHEEL_LINEAR_TO_ANGULAR;
    RadiansPerSecond leftOuter = (forward - deltaOuter) * WHEEL_LINEAR_TO_ANGULAR;
    RadiansPerSecond rightOuter = (forward + deltaOuter) * WHEEL_LINEAR_TO_ANGULAR;

    // It is possible for another node to request an invalid combination of linear and angular velocities that the rover can not realize
    // Instead of clipping, scale down based on the maximal speed to preserve the ratio

    RadiansPerSecond maximal = std::max(abs(leftOuter), abs(rightOuter));
    if (maximal > MAX_SPEED) {
        Dimensionless changeRatio = MAX_SPEED / maximal;
        leftInner = leftInner * changeRatio;
        rightInner = rightInner * changeRatio;
        leftOuter = leftOuter * changeRatio;
        rightOuter = rightOuter * changeRatio;
    }

    {
        Velocity leftVelocity;
        leftVelocity.names = LEFT_NAMES;
        leftVelocity.velocities = {leftOuter.get(), leftInner.get(), leftOuter.get()};
        leftVelocityPub.publish(leftVelocity);
    }
    {
        Velocity rightVelocity;
        rightVelocity.names = RIGHT_NAMES;
        rightVelocity.velocities = {rightOuter.get(), rightInner.get(), rightOuter.get()};
        rightVelocityPub.publish(rightVelocity);
    }
}
