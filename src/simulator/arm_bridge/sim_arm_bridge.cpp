#include "sim_arm_bridge.hpp"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

#include <boost/range/combine.hpp>

ros::Subscriber sub;

ros::Publisher jointStatePub;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "sim_arm_bridge");
    ros::NodeHandle nh;

    sub = nh.subscribe("arm_position", 1, positionCallback);

    ros::spin();
    return EXIT_SUCCESS;
}

void positionCallback(mrover::Position const& positions) {
    std::string_view name;
    float position;
    sensor_msgs::JointState jointState;
    for (auto const& tuple: boost::combine(positions.names, positions.positions)) {
        boost::tie(name, position) = tuple;

        jointState.name.emplace_back(name);
        jointState.position.emplace_back(position);
    }
    jointStatePub.publish(jointState);
}