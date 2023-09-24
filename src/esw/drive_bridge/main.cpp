#include "../motor_library/motors_manager.hpp"
#include <ros/ros.h>       // for ros and ROS_INFO

void ROSHandler::moveDrive(const geometry_msgs::Twist::ConstPtr& msg);

MotorsManager driveManager;
std::vector<std::string> driveNames = 
    {"FrontLeft", "FrontRight", "MiddleLeft", "MiddleRight", "BackLeft", "BackRight"};

std::optional<float> WHEEL_DISTANCE_INNER;
std::optional<float> WHEEL_DISTANCE_OUTER;
std::optional<float> WHEELS_M_S_TO_MOTOR_REV_S;
std::optional<float> MAX_MOTOR_SPEED_REV_S;

int main(int argc, char** argv) {

    ros::init(argc, argv, "drive_bridge");
    ros::NodeHandle nh;

    std::vector<std::string> driveNames = 
        {"FrontLeft", "FrontRight", "MiddleLeft", "MiddleRight", "BackLeft", "BackRight"};

    XmlRpc::XmlRpcValue controllersRoot;
    nh.getParam("motors/controllers", controllersRoot);
    driveManager = MotorsManager(driveNames, controllersRoot);

    float roverWidth;
    float roverLength;
    nh.getParam("rover/width", roverWidth);
    nh.getParam("rover/length", roverLength);
    WHEEL_DISTANCE_INNER = roverWidth / 2.0
    WHEEL_DISTANCE_OUTER = sqrt(((roverWidth / 2.0) ** 2) + ((roverLength / 2.0) ** 2))

    float ratioMotorToWheel = rospy.get_param("wheel/gear_ratio")
    nh.getParam("wheel/gear_ratio", ratioMotorToWheel);
    // To convert m/s to rev/s, multiply by this constant. Divide by circumference, multiply by gear ratio.
    float wheelRadius;
    nh.getParam("wheel/radius", wheelRadius);
    WHEELS_M_S_TO_MOTOR_REV_S = (1 / (wheelRadius * 2 * M_PI)) * ratioMotorToWheel;

    float maxSpeedMPerS = rospy.get_param("rover/max_speed")
    nh.getParam("rover/max_speed", maxSpeedMPerS);
    assert(maxSpeedMPerS > 0);

    MAX_MOTOR_SPEED_REV_S = maxSpeedMPerS * self.WHEELS_M_S_TO_MOTOR_REV_S

    ros::Subscriber moveDriveSubscriber = n->subscribe<sensor_msgs::JointState>("ra_cmd", 1, moveDrive);;

    ROS_INFO("Initialization Done. \nLooping. \n");

    ros::spin();

    return 0;
}

void moveDrive(const geometry_msgs::Twist::ConstPtr& msg) {

    float forward = msg->linear.x;
    float turn = msg->angular.z;

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
        const std::string& name = pair.first;
        float velocity = pair.second;

        // TODO - account for multipliers
        float multiplier = 1;
        velocity *= multiplier;

        Controller& controller = driveManager.get_controller(name);
        controller.set_desired_speed(velocity);
    }
}
