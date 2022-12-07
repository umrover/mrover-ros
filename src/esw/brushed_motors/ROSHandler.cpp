#include "ROSHandler.h"

// REQUIRES: rosNode is a pointer to the created node.
// MODIFIES: static variables
// EFFECTS: Initializes all subscribers and publishers.
void ROSHandler::init(ros::NodeHandle* rosNode) {

    n = rosNode;

    RANames = {"joint_a", "joint_b", "joint_c", "joint_d", "joint_e", "joint_f", "finger", "gripper"};

    openLoopSubscriberRA = n->subscribe<sensor_msgs::JointState>(
            "ra_cmd",
            1,
            moveOpenLoopRACommand);

    openLoopSubscriberMast = n->subscribe<mrover::GimbalCmd>("gimbal_cmd", 1, moveGimbal);

    jointData.name = RANames;
    jointData.position = std::vector<double>(RANames.size(), 0);
    jointData.velocity = std::vector<double>(RANames.size(), std::nan(""));
    jointData.effort = std::vector<double>(RANames.size(), std::nan(""));

    jointDataPublisherRA = n->advertise<sensor_msgs::JointState>("ra_data", 1);
}

// REQUIRES: name is a valid name
// MODIFIES: nothing
// EFFECTS: Moves the RA joints in open loop
// and publishes angle data right after.
void ROSHandler::moveOpenLoopRACommand(const sensor_msgs::JointState::ConstPtr& msg) {

    for (size_t i = 0; i < RANames.size(); ++i) {
        auto controller_iter = ControllerMap::controllersByName.find(RANames[i]);
        assert(controller_iter != ControllerMap::controllersByName.end());

        auto name = controller_iter->first;
        auto controller = controller_iter->second;

        controller->moveOpenLoop((float) msg->velocity[i]);
        jointData.position[i] = controller->getCurrentAngle();
    }

    jointDataPublisherRA.publish(jointData);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves a gimbal.
void ROSHandler::moveGimbal(const mrover::GimbalCmd::ConstPtr& msg) {
    ControllerMap::controllersByName["mast_up_down"]->moveOpenLoop((float) msg->up_down);
    ControllerMap::controllersByName["mast_left_right"]->moveOpenLoop((float) msg->left_right);
}