#include <ros/ros.h>

#include <iostream>
#include <motors_manager.hpp>
#include <units/units.hpp>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "brushless_test_bridge");
    ros::NodeHandle nh;
    ROS_INFO("Running");

    // To get it to run:
    // - ./fdcanusb_daemon -F -v /dev/fdcanusb vcan0
    // - sudo ip link set vcan0 up
    // - rosparam load config/esw_devboard.yaml
    // - rosrun mrover can_driver_node _interface:=vcan0
    // - roslaunch brushless_test.launch

    auto brushlessController = std::make_unique<mrover::BrushlessController>(nh, "jetson", "devboard");

    int count = -100;
    ros::Rate rate{2};
    while (ros::ok()) {
        // Throttle test
        //brushlessController->setDesiredThrottle(mrover::Percent{(float) count / 100.0});
        //brushlessController->setDesiredVelocity(mrover::RadiansPerSecond{10.0});
        brushlessController->setDesiredPosition(mrover::Radians{(float) count / 10.0});
        count++;
        if (count == 101) break;
        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
