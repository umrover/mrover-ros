#include "messaging.hpp"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <mrover/LED.h>

#include <chrono>
#include <stdexcept>
#include <thread>

void sleep(int ms);
void set_arm_laser_enable(ros::ServiceClient& client, bool enable);
void set_auton_led(ros::Publisher& publisher, bool red, bool green, bool blue, bool blinking);

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "science_test_bridge");
    ros::NodeHandle nh;

    /* Arm Laser Test */
    // ros::service::waitForService("enable_arm_laser");
    // ros::ServiceClient service_client = nh.serviceClient<std_srvs::SetBool::Request, std_srvs::SetBool::Response>("enable_arm_laser");

    // ROS_INFO("****BEGIN ARM LASER TEST****");

    // try {
    //     ROS_INFO("Enabling arm laser...");
    //     set_arm_laser_enable(service_client, true);

    //     sleep(1000);

    //     ROS_INFO("Disabling arm laser...");
    //     set_arm_laser_enable(service_client, false);
    // } catch (std::exception const& e) {
    //     ROS_ERROR("Arm laser test FAILED: %s", e.what());
    //     return 1;
    // }

    ROS_INFO("****END ARM LASER TEST****");
    ros::Rate rate(10);
    while (ros::ok()) {
        /* Auton LED Test */
        ROS_INFO("****BEGIN AUTON LED TEST****");
        ros::Publisher ledPublisher = nh.advertise<mrover::LED>("led", 1);

        set_auton_led(ledPublisher, true, false, false, false);
        ROS_INFO("RED SOLID");
        sleep(10000);

        ROS_INFO("RED BLINKING");
        set_auton_led(ledPublisher, true, false, false, true);
        sleep(10000);

        ROS_INFO("GREEN SOLID");
        set_auton_led(ledPublisher, false, true, false, false);
        sleep(10000);

        ROS_INFO("GREEN BLINKING");
        set_auton_led(ledPublisher, false, true, false, true);
        sleep(10000);

        ROS_INFO("BLUE SOLID");
        set_auton_led(ledPublisher, false, false, true, false);
        sleep(10000);

        ROS_INFO("BLUE BLINKING");
        set_auton_led(ledPublisher, false, false, true, true);
        sleep(10000);

        set_auton_led(ledPublisher, false, false, false, false);
        ROS_INFO("****END AUTON LED TEST****");
    }

    return 0;
}

void sleep(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void set_arm_laser_enable(ros::ServiceClient& client, bool enable) {
    std_srvs::SetBool::Request request;
    request.data = enable;

    if (std_srvs::SetBool::Response resp; client.call(request, resp)) {
        ROS_INFO("Response: %s", resp.message.c_str());
    } else {
        ROS_ERROR("Failed to call service");
        throw std::runtime_error(resp.message);
    }
}

void set_auton_led(ros::Publisher& publisher, bool red, bool green, bool blue, bool blinking) {
    mrover::LED ledMessage{};
    ledMessage.red = red;
    ledMessage.green = green;
    ledMessage.blue = blue;
    ledMessage.is_blinking = blinking;
    publisher.publish(ledMessage);
}