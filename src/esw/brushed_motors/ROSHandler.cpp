#include "ROSHandler.h"

// REQUIRES: root is created from calling ros::param::get("motors/controllers", root)
// and rosNode is a pointer to the created node.
// MODIFIES: n, subscribersByName, and publishersByName.
// EFFECTS: Initializes all subscribers and publishers.
void ROSHandler::init(XmlRpc::XmlRpcValue& root, ros::NodeHandle* rosNode) {

    n = rosNode;

    for (int32_t i = 0; i < root.size(); ++i) {
        assert(root[i].hasMember("name") &&
               root[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string name = static_cast<std::string>(root[i]["name"]);

        assert(root[i].hasMember("open_loop_topic") &&
               root[i]["open_loop_topic"].getType() == XmlRpc::XmlRpcValue::TypeString);
        std::string openLoopTopic = static_cast<std::string>(root[i]["open_loop_topic"]);

        openLoopSubscribersByName[name] =
                n->subscribe<sensor_msgs::JointState>(
                        openLoopTopic,
                        1,
                        boost::bind(moveJointOpenLoopCommand, _1, name));

        if (root[i].hasMember("joint_data_topic") &&
            root[i]["joint_data_topic"].getType() == XmlRpc::XmlRpcValue::TypeString) {
            std::string publishTopic = static_cast<std::string>(root[i]["joint_data_topic"]);
            jointDataPublishersByName[name] =
                    n->advertise<sensor_msgs::JointState>(publishTopic, 1);
        }
    }
}

// REQUIRES: name is a valid name
// MODIFIES: nothing
// EFFECTS: Moves a joint in open loop
// and publishes angle data right after.
void ROSHandler::moveJointOpenLoopCommand(
        sensor_msgs::JointState& state,
        std::string& name) {
    ControllerMap::controllersByName[name]->moveOpenLoop(state.velocity[0]);

    float jointAngle = ControllerMap::controllersByName[name]->getCurrentAngle();

    if (jointDataPublishersByName.find(name) != jointDataPublishersByName.end()) {
        sensor_msgs::JointState msg;
        msg.position.push_back(jointAngle);
        jointDataPublishersByName[name].publish(msg);
    }
}