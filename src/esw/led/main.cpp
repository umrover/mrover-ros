#include <mrover/CAN.h>
#include <mrover/LED.h>
#include <mrover/StateMachineStateUpdate.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>

static std::string const DONE_STATE = "DoneState";

ros::Publisher LEDPublisher;

enum class LEDMode {
    Unknown = 0,
    Off = 1,
    Red = 2,
    BlinkingGreen = 3,
    Blue = 4
};

bool navDone = false;
bool teleop_enabled = false;
LEDMode led_mode = LEDMode::Unknown;

auto update_led() -> void {
    if (teleop_enabled) {
        led_mode = LEDMode::Blue;
    } else if (navDone) {
        led_mode = LEDMode::BlinkingGreen;
    } else {
        led_mode = LEDMode::Red;
    }

    mrover::LED led_msg;
    led_msg.red = led_mode == LEDMode::Red;
    led_msg.green = led_mode == LEDMode::BlinkingGreen;
    led_msg.blue = led_mode == LEDMode::Blue;
    LEDPublisher.publish(led_msg);
}

auto state_machine_state_update(mrover::StateMachineStateUpdate::ConstPtr const& msg) -> void {
    navDone = msg->state == DONE_STATE;

    update_led();
}

auto teleop_enabled_update(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response) -> bool {
    teleop_enabled = request.data;

    update_led();

    return response.success = true;
}

auto main(int argc, char** argv) -> int {
    // Initialize the ROS node
    ros::init(argc, argv, "led");
    ros::NodeHandle nh;

    LEDPublisher = nh.advertise<mrover::CAN>("led", 1);
    ros::ServiceServer teleopClient = nh.advertiseService("teleop_enabled", teleop_enabled_update);
    ros::Subscriber stateMachineStateSubscriber = nh.subscribe<mrover::StateMachineStateUpdate>("nav_state", 1, state_machine_state_update);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}
