#include <ros/rate.h>
#include <ros/ros.h>

#include <iostream>
#include <motors_group.hpp>
#include <units/units.hpp>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "brushless_test_bridge");
    ros::NodeHandle nh;
    ROS_INFO("Running");

    // To get it to run:
    //  On laptop:
    // - ./fdcanusb_daemon -F -v /dev/fdcanusb vcan0
    // - sudo ip link set vcan0 up
    //  On Jetson:
    // - roscore
    // - sudo ip link set can0 up type can bitrate 500000 dbitrate 1000000 berr-reporting on fd on restart-ms 100
    // - rosparam load config/esw_devboard.yaml
    // - rosrun mrover can_driver_node _interface:=can0
    // - roslaunch brushless_test.launch

    // auto brushlessController_de0 = std::make_unique<mrover::BrushlessController>(nh, "jetson", "joint_de_0");
    // auto brushlessController_de1 = std::make_unique<mrover::BrushlessController>(nh, "jetson", "joint_de_1");
    
    // fake DE publisher:

    auto DEPub = std::make_unique<ros::Publisher>(nh.advertise<mrover::Velocity>("arm_velocity_cmd", 1));


    mrover::Velocity msg;
    msg.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"};
    msg.velocities = {0, 0, 0, 0, 10, 0, 0};

    // brushlessController_de0->setStop();
    // brushlessController_de1->setStop();

    ros::Rate rate{1};

    int count = 0;

    while(ros::ok()){
        // publish DE velocity:
        DEPub->publish(msg);

        count++;

        if(count > 50) {
            msg.velocities[0] *= -1;
            count = 0;
        }   

        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}

