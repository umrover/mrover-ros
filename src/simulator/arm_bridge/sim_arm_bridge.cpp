#include "sim_arm_bridge.hpp"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

namespace mrover {

    ros::Subscriber subscriber;

    ros::Publisher jointStatePublisher;

    int init(int argc, char* argv[]) {
        ros::init(argc, argv, "sim_arm_bridge");
        ros::NodeHandle nh;

        subscriber = nh.subscribe("arm_position_cmd", 1, positionCallback);

        ros::spin();
        return EXIT_SUCCESS;
    }

    void positionCallback(mrover::Position const& positions) {
        sensor_msgs::JointState jointState;
        jointState.name = positions.names;
        jointState.position.assign(positions.positions.begin(), positions.positions.end());
        jointStatePublisher.publish(jointState);
    }

} // namespace mrover

int main(int argc, char* argv[]) {
    return mrover::init(argc, argv);
}
