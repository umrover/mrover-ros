#include "messaging.hpp"
#include "ros/ros.h"
#include "std_srvs/SetBool.h"
#include <mrover/LED.h>
#include <mrover/Position.h>
#include <sensor_msgs/JointState.h>

#include <chrono>
#include <stdexcept>
#include <thread>

void sleep(int ms);
void set_arm_position(ros::Publisher& publisher, std::vector<float> positions, int delay);

std::vector<double> position_data;

auto arm_position_callback(sensor_msgs::JointState::ConstPtr const& msg) {
    ROS_INFO(";)");
    position_data = msg->position;
    ROS_INFO("hey");
}

void reset_pos(ros::Publisher armPublisher, int delay) {
    set_arm_position(armPublisher, {0, 0, 0, 0, 0, 0, 0}, delay);
}

void de_roll(float position, ros::Publisher armPublisher, int delay = 5000) {
    set_arm_position(armPublisher, {0, 0, 0, 0, 0, position, 0}, delay);
    ROS_INFO("Joint_DE_ROLL: %f", position_data[4]);
    reset_pos(armPublisher, delay);
}

void de_pitch(float position, ros::Publisher armPublisher, int delay = 5000) {
    set_arm_position(armPublisher, {0, 0, 0, position, 0, 0, 0}, delay);
    ROS_INFO("Joint_DE_PITCH: %f", position_data[3]);
    reset_pos(armPublisher, delay);
}

auto main(int argc, char** argv) -> int {
    // Delay between operations
    int delay = 5000;
    position_data.resize(7);

    ros::init(argc, argv, "arm_position_test_bridge");
    ros::NodeHandle nh;
    ros::Rate rate(10);

    ros::Publisher armPublisher = nh.advertise<mrover::Position>("arm_position_cmd", 1);
    ros::Subscriber armSubscriber = nh.subscribe<sensor_msgs::JointState>("arm_joint_data", 10, arm_position_callback);

    while (ros::ok()) {
        /* Arm Position Test */
        ROS_INFO("****BEGIN BASIC AUTON ARM POSITION TEST****");


        // ROS_INFO("MOVE A");

        // set_arm_position(armPublisher, {0.4, 0, 0, 0, 0, 0, 0}, delay);
        // ROS_INFO("Joint_A: %f", position_data[0]);
        //reset_pos(armPublisher, delay);

        // Test forwards limit switch
        //set_arm_position(armPublisher, {1, 0, 0, 0, 0, 0, 0}, delay);
        //ROS_INFO("Joint_A: %f", position_data[0]);
        //reset_pos(armPublisher, delay);

        // Test backwards limit switch
        //set_arm_position(armPublisher, {-0.4, 0, 0, 0, 0, 0, 0}, delay);
        //ROS_INFO("Joint_A: %f", position_data[0]);
        //reset_pos(armPublisher, delay);


        // ROS_INFO("MOVE B");
        // set_arm_position(armPublisher, {0, -0.70, 0, 0, 0, 0, 0}, delay);
        // ROS_INFO("Joint_B: %f", position_data[1]);
        // reset_pos(armPublisher, delay);

        ROS_INFO("MOVE C");
        set_arm_position(armPublisher, {0, 0, 2, 0, 0, 0, 0}, delay);
        ROS_INFO("Joint_C: %f", position_data[2]);
        reset_pos(armPublisher, delay);

        // set_arm_position(armPublisher, {0, 0, 1.7, 0, 0, 0, 0}, delay);
        // ROS_INFO("Joint_C: %f", position_data[2]);
        // reset_pos(armPublisher, delay);

        // ROS_INFO("MOVE DE PITCH");
        // float test_de_pitch_positions[4] = {2.35, -2.35, 1.17, -1.17};
        // for(float value : test_de_pitch_positions) {
        //     de_pitch(value, armPublisher);
        // }

        // ROS_INFO("MOVE DE ROLL");
        // float test_de_roll_positions[4] = {6.28, -6.28, 3.14, -3.14};
        // for(float value : test_de_roll_positions){
        //     de_roll(value, armPublisher);
        // }
        ROS_INFO("****END BASIC ARM POSITION TEST****");
        //     ROS_INFO("****BEGIN COMPLEX AUTON ARM POSITION TEST****");

        //     ROS_INFO("BOTH DE");
        //     for(int i = 0; i < 4; i++){
        //         set_arm_position(armPublisher, {0,0,0,0,test_de_pitch_positions[i],test_de_roll_positions[i],0}, delay);
        //         ROS_INFO("Joint_DE_ROLL: %f", position_data[4]);
        //         ROS_INFO("Joint_DE_PITCH: %f", position_data[3]);
        //         reset_pos(armPublisher, delay);
        //     }

        //     // Joint A: [0.0, 1.0]
        //     // Joint B: [-0.78540, 0.0]
        //     // Joint C: [-2.094, 1.745] 220 degrees of motion
        //     // Joint DE-Pitch: [-2.268, 2.268]
        //     // Joint DE-Roll: [-inf,inf]
        //     ROS_INFO("A,B,C");
        //     //min position
        //     set_arm_position(armPublisher, {0,-0.78,-2.0,0,0,0,0}, delay);
        //     ROS_INFO("Joint_A: %f", position_data[0]);
        //     ROS_INFO("Joint_B: %f", position_data[1]);
        //     ROS_INFO("Joint_C: %f", position_data[2]);
        //     reset_pos(armPublisher, delay);

        //     // max position
        //     set_arm_position(armPublisher, {0.9,0.0,1.7,0,0,0,0}, delay);
        //     ROS_INFO("Joint_A: %f", position_data[0]);
        //     ROS_INFO("Joint_B: %f", position_data[1]);
        //     ROS_INFO("Joint_C: %f", position_data[2]);
        //     reset_pos(armPublisher, delay);

        //     set_arm_position(armPublisher, {0.3,-0.32,-1.2,0,0,0,0}, delay);
        //     ROS_INFO("Joint_A: %f", position_data[0]);
        //     ROS_INFO("Joint_B: %f", position_data[1]);
        //     ROS_INFO("Joint_C: %f", position_data[2]);
        //     ROS_INFO("****END COMPLEX AUTON ARM POSITION TEST****");
    }

    return 0;
}

void sleep(int ms) {
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void set_arm_position(ros::Publisher& publisher, std::vector<float> positions, int delay) {
    mrover::Position armMsg;
    // TODO: figure out how to set allen_key and gripper to nan
    armMsg.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"};
    armMsg.positions = std::move(positions);
    ROS_INFO("[0]: %f, [1]: %f, [2]: %f", armMsg.positions[0], armMsg.positions[1], armMsg.positions[2]);

    for (int i = 0; i < delay; i += 50) {
        sleep(50);
        publisher.publish(armMsg);
    }
}
