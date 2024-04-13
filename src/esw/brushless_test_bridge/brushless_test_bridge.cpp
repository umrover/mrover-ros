#include <ros/node_handle.h>
#include <ros/rate.h>
#include <ros/ros.h>

#include <iostream>
#include <motors_group.hpp>
#include <units/units.hpp>


// void test_joint_de(ros::NodeHandle& nh) {

//     auto brushlessController_de0 = std::make_unique<mrover::BrushlessController>(nh, "jetson", "joint_de_0");
//     auto brushlessController_de1 = std::make_unique<mrover::BrushlessController>(nh, "jetson", "joint_de_1");
    
//     brushlessController_de0->setStop();
//     brushlessController_de1->setStop();
    
//     ros::Rate rate{20};
//     while (ros::ok()) {
//         brushlessController_de0->setDesiredVelocity(mrover::RadiansPerSecond{60.0});
//         brushlessController_de1->setDesiredVelocity(mrover::RadiansPerSecond{60.0});
        
//         ros::spinOnce();
//         rate.sleep();
//     }

    
// }

auto main(int argc, char** argv) -> int {
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
    // auto SAPub = std::make_unique<ros::Publisher>(nh.advertise<mrover::Velocity>("sa_velocity_cmd", 1));


    mrover::Velocity armMsg;
    mrover::Velocity saMsg;
    armMsg.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"};
    armMsg.velocities = {0, 0, 0, 0, -10, 0, 0};

    // saMsg.names = {"sa_x", "sa_y", "sa_z", "sampler", "sensor_actuator"};
    // saMsg.velocities = {0, 0, 0.07,0, 0};

    // brushlessController_de0->setStop();
    // brushlessController_de1->setStop();

    ros::Rate rate{15};

    int count = 0;

    
    while(ros::ok()){
        // publish DE velocity:
        DEPub->publish(armMsg);
        // SAPub->publish(saMsg);
        count++;

        if(count > 100) {
            armMsg.velocities[3] *= -1;
            armMsg.velocities[4] *= -1;
            count = 0;
        }   

        ros::spinOnce();
        rate.sleep();
    }
    

    return EXIT_SUCCESS;
}

