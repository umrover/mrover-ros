#include "sim_arm_bridge.hpp"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

namespace mrover {

    // Subscribers

    ros::Subscriber subscriber;

    // Publishers

    ros::Publisher join_state_publisher;

    // Private state

    uint32_t seq = 0;
    sensor_msgs::JointState joint_state;

    int run(int argc, char* argv[]) {
        ros::init(argc, argv, "sim_arm_bridge");
        ros::NodeHandle nh;

        join_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
        subscriber = nh.subscribe("arm_position_cmd", 1, positionCallback);

        ros::spin();
        return EXIT_SUCCESS;
    }

    void positionCallback(mrover::Position const& positions) {
        joint_state.name = positions.names;
        joint_state.position.assign(positions.positions.begin(), positions.positions.end());
        joint_state.header.frame_id = "base_link";
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.seq = seq++;
        join_state_publisher.publish(joint_state);
    }

} // namespace mrover

int main(int argc, char* argv[]) {
    return mrover::run(argc, argv);
}
