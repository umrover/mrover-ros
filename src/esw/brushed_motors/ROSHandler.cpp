#include "ROSHandler.h"

// REQUIRES: rosNode is a pointer to the created node.
// MODIFIES: n, subscribersByName, and publishersByName.
// EFFECTS: Initializes all subscribers and publishers.
void ROSHandler::init(ros::NodeHandle* rosNode) {

    n = rosNode;

    std::vector<subscriberData> openLoopSubscribers = {
            {&openLoopSubscriberRAJointA, "open_loop/ra/joint_a", "ra_joint_a"},
            {&openLoopSubscriberRAJointB, "open_loop/ra/joint_b", "ra_joint_b"},
            {&openLoopSubscriberRAJointC, "open_loop/ra/joint_c", "ra_joint_c"},
            {&openLoopSubscriberRAJointD, "open_loop/ra/joint_d", "ra_joint_d"},
            {&openLoopSubscriberRAJointE, "open_loop/ra/joint_e", "ra_joint_e"},
            {&openLoopSubscriberRAJointF, "open_loop/ra/joint_f", "ra_joint_f"},
            {&openLoopSubscriberRAFinger, "open_loop/ra/finger", "ra_finger"},
            {&openLoopSubscriberRAGripper, "open_loop/ra/gripper", "ra_gripper"}};

    std::vector<publisherData> jointDataPublishers = {
            {&jointDataPublisherJointA, "joint_data/ra/joint_a", "ra_joint_a"},
            {&jointDataPublisherJointB, "joint_data/ra/joint_b", "ra_joint_b"},
            {&jointDataPublisherJointC, "joint_data/ra/joint_c", "ra_joint_c"},
            {&jointDataPublisherJointD, "joint_data/ra/joint_d", "ra_joint_d"},
            {&jointDataPublisherJointE, "joint_data/ra/joint_e", "ra_joint_e"},
            {&jointDataPublisherJointF, "joint_data/ra/joint_f", "ra_joint_f"}};

    for (subscriberData& subData: openLoopSubscribers) {
        *(subData.subscriber) =
                n->subscribe<sensor_msgs::JointState>(
                        subData.topic,
                        1,
                        [capture0 = subData.name](auto && PH1) { return moveJointOpenLoopCommand(std::forward<decltype(PH1)>(PH1), capture0); });
    }

    openLoopSubscriberMast = n->subscribe<mrover::GimbalCmd>("gimbal_control", 1, moveGimbal);

    for (publisherData& pubData: jointDataPublishers) {
        *(pubData.publisher) =
                n->advertise<sensor_msgs::JointState>(pubData.topic, 1);
        jointDataPublishersByName[pubData.name] = pubData.publisher;
    }
}

// REQUIRES: name is a valid name
// MODIFIES: nothing
// EFFECTS: Moves a joint in open loop
// and publishes angle data right after.
void ROSHandler::moveJointOpenLoopCommand(
        const sensor_msgs::JointState::ConstPtr& state,
        const std::string& name) {
    ControllerMap::controllersByName[name]->moveOpenLoop((float)state->velocity[0]);

    float jointAngle = ControllerMap::controllersByName[name]->getCurrentAngle();

    if (jointDataPublishersByName.find(name) != jointDataPublishersByName.end()) {
        sensor_msgs::JointState msg;
        msg.position.push_back(jointAngle);
        jointDataPublishersByName[name]->publish(msg);
    }
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Moves a gimbal.
void ROSHandler::moveGimbal(const mrover::GimbalCmd::ConstPtr& msg) {
    ROS_INFO("Initialization Done. Looping. \n");
    ControllerMap::controllersByName["mast_up_down"]->moveOpenLoop((float)msg->up_down);
    ControllerMap::controllersByName["mast_left_right"]->moveOpenLoop((float)msg->left_right);
}