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
    Position target;
    Position current;
    float ANGULAR_VELOCITY = 1;
    float LINEAR_VELOCITY = 10;

    int run(int argc, char* argv[]) {
        ros::init(argc, argv, "sim_arm_bridge");
        ros::NodeHandle nh;

        join_state_publisher = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
        subscriber = nh.subscribe("arm_position_cmd", 1, positionCallback);

        //ros::spin();

        for (int i = 0; i < 5; i++)
            current.positions.push_back(0);

        float freq = 100;
        ros::Rate rate(freq); // definitely a better way to do this
        float dt = 1 / freq;
        while (ros::ok()) {
            if (!target.names.empty()) { // do nothing if we haven't gotten a target yet
                current.names = target.names;
                // current.positions = target.positions;
                for (int i = 0; i < target.positions.size(); i++) {
                    float sign = target.positions.at(i) < current.positions.at(i) ? -1 : 1;
                    if (i == 0) { // linear joint
                        current.positions.at(i) = current.positions.at(i) + sign * std::min(LINEAR_VELOCITY * dt, abs(target.positions.at(i) - current.positions.at(i)));
                    } else {
                        current.positions.at(i) = current.positions.at(i) + sign * std::min(ANGULAR_VELOCITY * dt, abs(target.positions.at(i) - current.positions.at(i)));
                    }
                    // current.positions.at(i) = target.positions.at(i);
                }

                joint_state.name = current.names;
                joint_state.position.assign(current.positions.begin(), current.positions.end());
                joint_state.header.frame_id = "base_link";
                joint_state.header.stamp = ros::Time::now();
                joint_state.header.seq = seq++;
                join_state_publisher.publish(joint_state);
            }
            rate.sleep();
            ros::spinOnce();
        }

        return EXIT_SUCCESS;
    }

    void positionCallback(mrover::Position const& positions) {
        // joint_state.name = positions.names;
        // joint_state.position.assign(positions.positions.begin(), positions.positions.end());
        // joint_state.header.frame_id = "base_link";
        // joint_state.header.stamp = ros::Time::now();
        // joint_state.header.seq = seq++;
        // join_state_publisher.publish(joint_state);
        target = positions;
    }

    void move() {
        // ros::Rate rate(100); // definitely a better way to do this
        // while (ros::ok()) {
        //     // for (int i = 0; i < current.positions.size(); i++) {
        //     // float sign = target.positions.at(i) < current.positions.at(i) ? -1 : 1;
        //     // if (i == 0) { // linear joint
        //     // current.positions.at(i) = current.positions.at(i) + sign * std::min(LINEAR_VELOCITY, abs(target.positions.at(i) - current.positions.at(i)));
        //     // } else {
        //     // current.positions.at(i) = current.positions.at(i) + sign * std::min(ANGULAR_VELOCITY, abs(target.positions.at(i) - current.positions.at(i)));
        //     // }
        //     // }
        //     current = target;

        //     joint_state.name = current.names;
        //     joint_state.position.assign(current.positions.begin(), current.positions.end());
        //     joint_state.header.frame_id = "base_link";
        //     joint_state.header.stamp = ros::Time::now();
        //     joint_state.header.seq = seq++;
        //     join_state_publisher.publish(joint_state);
        //     rate.sleep();
        // }
    }

} // namespace mrover

int main(int argc, char* argv[]) {
    return mrover::run(argc, argv);
    // mrover::run(argc, argv);
    // mrover::move();
}
