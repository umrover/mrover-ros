#pragma once

#include "ControllerMap.h"          // for ControllerMap
#include <ros/ros.h>                // for ros
#include <sensor_msgs/JointState.h> // for JointState
#include <unordered_map>            // for unordered_map

#define NOW std::chrono::high_resolution_clock::now()

/*
ROSHandler.h is responsible for handling incoming and outgoing ROS messages.
Incoming ROS messages will trigger functions which call the functions on the appropriate virtual Controllers. 
Outgoing ROS messages are triggered by a clock, which query the functions on the appropriate virtual Controllers for data.
*/
class ROSHandler {
private:
    // This holds the ROS Node.
    static ros::NodeHandle* n;

    // This keeps track of all the open loop subscribers
    static std::unordered_map<std::string, ros::Subscriber> openLoopSubscribersByName;

    // This keeps track of all the angle data publishers
    static std::unordered_map<std::string, ros::Publisher> angleDataPublishersByName;

    // REQUIRES: name is a valid name
    // MODIFIES: nothing
    // EFFECTS: Moves a joint in open loop
    // and publishes angle data right after.
    static void moveJointOpenLoopCommand(
            sensor_msgs::JointState& state,
            std::string& name);

public:
    // REQUIRES: root is created from calling ros::param::get("motors/controllers", root)
    // MODIFIES: n, subscribersByName, and publishersByName.
    // EFFECTS: Initializes all subscribers and publishers.
    static void init(XmlRpc::XmlRpcValue& root);
};
