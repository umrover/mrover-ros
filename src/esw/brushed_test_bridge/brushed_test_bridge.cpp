#include <ros/init.h>
#include <ros/node_handle.h>

#include <motors_group.hpp>

using namespace mrover;

int main(int argc, char** argv) {
    ros::init(argc, argv, "brushed_test_bridge");
    ros::NodeHandle nh;

    auto brushlessController = std::make_unique<BrushedController>(nh, "jetson", "devboard");

    Radians target = 10_rad;

    ros::Timer timer = nh.createTimer(ros::Duration{5}, [&](ros::TimerEvent const&) {
        target *= -1;
        ROS_INFO_STREAM(std::format("Target: {}", target.get()));
    });

    ros::Rate rate(100);
    while (ros::ok()) {
        // brushlessController->setDesiredPosition(target);
        // brushlessController->setDesiredVelocity(RadiansPerSecond{2.0});
        brushlessController->setDesiredThrottle(-0.5);
        rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
