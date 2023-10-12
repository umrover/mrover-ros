#include "sim_arm_bridge.hpp"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

ros::Subscriber subscriber;

ros::Publisher jointStatePublisher;

constexpr std::string_view ARM_THROTTLE_CMD_TOPIC = "arm_throttle_cmd";
constexpr std::string_view ARM_VELOCITY_CMD_TOPIC = "arm_velocity_cmd";
constexpr std::string_view ARM_POSITION_CMD_TOPIC = "arm_position_cmd";

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "sim_arm_bridge");
    ros::NodeHandle nh;

    subscriber = nh.subscribe(std::string{ARM_POSITION_CMD_TOPIC}, 1, positionCallback);

    ros::spin();
    return EXIT_SUCCESS;
}

void positionCallback(mrover::Position const& positions) {
    sensor_msgs::JointState jointState;
    jointState.name = positions.names;
    jointState.position.assign(positions.positions.begin(), positions.positions.end());
    jointStatePublisher.publish(jointState);
}