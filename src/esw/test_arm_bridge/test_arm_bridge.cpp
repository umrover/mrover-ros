#include "mrover/Velocity.h"
#include <iostream>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <mrover/Throttle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_arm_bridge");
    ros::NodeHandle nh;

    ros::Publisher ros_pub = nh.advertise<mrover::Velocity>("arm_velocity_cmd", 1);
    mrover::Velocity throttle_cmd;
    throttle_cmd.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"};
    throttle_cmd.velocities = {0, 0, 0, 0, 10.0, 0, 0};

    ros::Rate rate(10);
    while (ros::ok()) {
        ros_pub.publish(throttle_cmd);
        rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
