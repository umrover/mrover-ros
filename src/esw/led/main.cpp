#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <mrover/LED.h>
#include <cstdint>
#include <mrover/CAN.h>

void changeTeleopEnabled(const std_msgs::Bool::ConstPtr& msg);
void changeAutonEnabled(const std_msgs::Bool::ConstPtr& msg);
void changeAutonCompleted(const mrover::NavState::ConstPtr& msg);

ros::Publisher LEDPublisher;

enum class LEDMode {
    Unknown = 0,
    Off = 1,
    Red = 2,
    BlinkingGreen = 3,
    Blue = 4
};

bool teleop_enabled  = true;
bool auton_enabled = false;
bool auton_completed = false;

void update_led() {
    static LEDMode led_mode = Unknown;
    LEDMode next_led_mode = Unknown;
    if (teleop_enabled) {
        next_led_mode = Blue;
    }
    else if (auton_enabled && auton_completed) {
        next_led_mode = BlinkingGreen;
    }
    else {
        next_led_mode = Red;
    }

    if (next_led_mode != led_mode) {
        led_mode = next_led_mode;
        mrover::LED led_msg;
        led_msg.red = led_mode == Red;
        led_msg.green = led_mode == Green;
        led_msg.blue = led_mode == Blue;
        led_msg.red = led_mode == Red;
        LEDPublisher.publish(led_msg);
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "led");
    ros::NodeHandle nh;

    LEDPublisher = nh.advertise<mrover::CAN>("led", 1);
    ros::Subscriber teleopEnabledSubscriber = nh.subscribe<std_msgs::Bool>("teleop_enabled", 1, changeTeleopEnabled);
    ros::Subscriber autonEnabledSubscriber = nh.subscribe<std_msgs::Bool>("auton_enabled", 1, changeAutonEnabled);
    ros::Subscriber autonCompletedSubscriber = nh.subscribe<mrover::NavState>("nav_status", 1, changeAutonCompleted);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void changeTeleopEnabled(const std_msgs::Bool::ConstPtr& msg) {
    next_teleop_enabled = msg->data;
    if (next_teleop_enabled != teleop_enabled) {
        teleop_enabled = next_teleop_enabled;
        update_led();
    }
}
void changeAutonEnabled(const std_msgs::Bool::ConstPtr& msg) {
    next_auton_enabled = msg->data;
    if (next_auton_enabled != auton_enabled) {
        auton_enabled = next_auton_enabled;
        update_led();
    }
}
void changeAutonCompleted(const mrover::NavState::ConstPtr& msg) {
    next_auton_completed = msg->completed;
    if (next_auton_completed != auton_completed) {
        auton_completed = next_auton_completed;
        update_led();
    }
}