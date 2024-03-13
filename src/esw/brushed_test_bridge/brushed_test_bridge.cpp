#include <ros/init.h>
#include <ros/node_handle.h>

#include <motors_group.hpp>

using namespace mrover;

int main(int argc, char** argv) {
    ros::init(argc, argv, "brushed_test_bridge");
    ros::NodeHandle nh;

    auto brushlessController0 = std::make_unique<BrushedController>(nh, "jetson", "gripper");
    auto brushlessController1 = std::make_unique<BrushedController>(nh, "jetson", "allen_key");

    auto targetPosition = 10_rad;
    auto targetVelocity = RadiansPerSecond{0.8};
    Dimensionless targetThrottle = 0.5;

    ros::Timer timer = nh.createTimer(ros::Duration{5}, [&](ros::TimerEvent const&) {
        targetPosition *= -1;
        targetVelocity *= -1;
        targetThrottle *= -1;
        ROS_INFO_STREAM(std::format("Target: {}", targetPosition.get()));
    });

    ros::Rate rate(100);
    while (ros::ok()) {
        // brushlessController->setDesiredPosition(targetPosition);
        // brushlessController->setDesiredVelocity(targetVelocity);
        brushlessController0->setDesiredThrottle(targetThrottle);
        brushlessController1->setDesiredThrottle(-1 * targetThrottle);
        rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
