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

    auto brushlessController = std::make_unique<mrover::BrushlessController>(nh, "jetson", "joint_de_1");

    int count = 0;
    ros::Rate rate{100};
    std::array<float, 4> positions = {1.0, 2.0, 3.0, 4.0};
    while (ros::ok()) {
        // Throttle test
        //brushlessController->setDesiredThrottle(mrover::Percent{((float) count) / 500.0});
        brushlessController->setDesiredVelocity(mrover::RadiansPerSecond{5.0});
        // brushlessController->setDesiredPosition(mrover::Radians{positions.at(count / 400)});
        count++;
        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
