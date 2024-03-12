#include "messaging.hpp"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <mrover/LED.h>

#include <chrono>
#include <stdexcept>
#include <thread>

void sleep(int ms);
void set_arm_position(ros::Publisher& publisher, float position);

float position_data[7] = {0, 0, 0, 0, 0, 0, 0};

auto arm_position_callback(Position::ConstPtr const& msg) {
    position_data = msg->position;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_position_test_bridge");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    while (ros::ok()) {
        /* Arm Position Test */
        ROS_INFO("****BEGIN AUTON ARM POSITION TEST****");
        ros::Publisher armPublisher = nh.advertise<mrover::Position>("arm_position_cmd", 1);
        ros::Subscriber armSubscriber = nh.subscribe<mrover::Position>("arm_joint_data", 10, arm_position_callback);
        
        ROS_INFO("MOVE A");
        set_arm_position(armPublisher, {0.4, 0, 0, 0, 0, 0, 0});
        ROS_INFO("Joint_A: %f", position_data[0]);
        sleep(5000);
        set_arm_position(armPublisher, {0, 0, 0, 0, 0, 0, 0});
        ROS_INFO("Joint_A: %f", position_data[0]);
        sleep(5000);

        ROS_INFO("MOVE B");
        set_arm_position(armPublisher, {0, -0.78, 0, 0, 0, 0, 0});
        ROS_INFO("Joint_B: %f", position_data[1]);
        sleep(5000);
        set_arm_position(armPublisher, {0, 0, 0, 0, 0, 0, 0});
        ROS_INFO("Joint_B: %f", position_data[1]);
        sleep(5000);

        ROS_INFO("MOVE C");
        set_arm_position(armPublisher, {0, 0, 0, 0, 0, 0, 0});
        ROS_INFO("Joint_C: %f", position_data[2]);
        sleep(5000);
        set_arm_position(armPublisher, {0, 0, 0, 0, 0, 0, 0});
        ROS_INFO("Joint_C: %f", position_data[2]);
        sleep(5000);

        ROS_INFO("MOVE DE PITCH");
        set_arm_position(armPublisher, {0, 0, 0, 0, 0, 0, 0});
        ROS_INFO("Joint_DE_PITCH: %f", position_data[3]);
        sleep(5000);
        set_arm_position(armPublisher, {0, 0, 0, 0, 0, 0, 0});
        ROS_INFO("Joint_DE_PITCH: %f", position_data[3]);
        sleep(5000);

        ROS_INFO("MOVE DE ROLL");
        set_arm_position(armPublisher, {0, 0, 0, 0, 0, 0, 0});
        ROS_INFO("Joint_DE_ROLL: %f", position_data[4]);
        sleep(5000);
        set_arm_position(armPublisher, {0, 0, 0, 0, 0, 0, 0});
        ROS_INFO("Joint_DE_ROLL: %f", position_data[4]);

        ROS_INFO("****END ARM POSITION TEST****");

        nh.spinOnce();
    }

    return 0;
}

void sleep(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void set_arm_position(ros::Publisher& publisher, float positions_arr[7]) {
    mrover::Position armMsg;
    // TODO: figure out how to set allen_key and gripper to nan
    armMsg.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"};
    armMsg.positions = positions_arr;
    publisher.publish(armMsg);
}