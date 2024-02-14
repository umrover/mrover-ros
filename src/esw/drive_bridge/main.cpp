#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <mrover/Velocity.h>

#include <unordered_map>
#include <can_device.hpp>
#include <motors_group.hpp>
#include <params_utils.hpp>

using namespace mrover;

void moveDrive(geometry_msgs::Twist::ConstPtr const& msg);

std::unique_ptr<MotorsGroup> driveManager;
std::vector<std::string> driveNames{"front_left", "front_right", "middle_left", "middle_right", "back_left", "back_right"};
std::unordered_map<std::string, ros::Publisher> drivePublishersByName;
Meters WHEEL_DISTANCE_INNER;
Meters WHEEL_DISTANCE_OUTER;
compound_unit<Radians, inverse<Meters>> WHEEL_LINEAR_TO_ANGULAR;
RadiansPerSecond MAX_MOTOR_SPEED;

auto main(int argc, char** argv) -> int {
    // Initialize the ROS node
    ros::init(argc, argv, "drive_bridge");
    ros::NodeHandle nh;

    for (auto name: driveNames) {
        drivePublishersByName[name] = nh.advertise<Velocity>(std::format("{}_velocity_cmd", name), 1);
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

    driveManager = std::make_unique<MotorsGroup>(nh, "drive");

    // Subscribe to the ROS topic for drive commands
    ros::Subscriber moveDriveSubscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, moveDrive);

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

    std::unordered_map<std::string, RadiansPerSecond> driveCommandVelocities{
            {"front_left", left},
            {"front_right", right},
            {"middle_left", left},
            {"middle_right", right},
            {"back_left", left},
            {"back_right", right},
    };

    for (auto [name, angularVelocity]: driveCommandVelocities) {
        Velocity msg;
        msg.names.push_back(name);
        msg.velocities.push_back(angularVelocity.get());
        drivePublishersByName.at(name).publish(msg);
    }
}
