#pragma once

#include "ControllerMap.h"          // for ControllerMap
#include <ros/ros.h>                // for ros
#include <sensor_msgs/JointState.h> // for JointState
#include <unordered_map>            // for unordered_map
#include <vector>                   // for vector

// A struct used for keeping track of subscriber data
struct subscriberData {
    ros::Subscriber* subscriber;
    std::string topic;
    std::string name;
};

// A struct used for keeping track of publisher data
struct publisherData {
    ros::Publisher* publisher;
    std::string topic;
    std::string name;
};

/*
ROSHandler.h is responsible for handling incoming and outgoing ROS messages.
Incoming ROS messages will trigger functions which call the functions
on the appropriate virtual Controllers. With each incoming ROS message,
if there exists a corresponding publisher, data will be published.
*/
class ROSHandler {
private:
    // This holds the ROS Node.
    inline static ros::NodeHandle* n;

    // This keeps track of all the open loop subscribers and publishers
    inline static ros::Subscriber openLoopSubscriberRAJointA;
    inline static ros::Subscriber openLoopSubscriberRAJointB;
    inline static ros::Subscriber openLoopSubscriberRAJointC;
    inline static ros::Subscriber openLoopSubscriberRAJointD;
    inline static ros::Subscriber openLoopSubscriberRAJointE;
    inline static ros::Subscriber openLoopSubscriberRAJointF;
    inline static ros::Subscriber openLoopSubscriberRAGripper;
    inline static ros::Subscriber openLoopSubscriberRAFinger;

    inline static ros::Publisher jointDataPublisherJointA;
    inline static ros::Publisher jointDataPublisherJointB;
    inline static ros::Publisher jointDataPublisherJointC;
    inline static ros::Publisher jointDataPublisherJointD;
    inline static ros::Publisher jointDataPublisherJointE;
    inline static ros::Publisher jointDataPublisherJointF;

    inline static std::unordered_map<std::string, ros::Publisher*> jointDataPublishersByName;

    // REQUIRES: name is a valid name
    // MODIFIES: nothing
    // EFFECTS: Moves a joint in open loop
    // and publishes angle data right after.
    static void moveJointOpenLoopCommand(
            const sensor_msgs::JointState::ConstPtr& state,
            const std::string& name);

public:
    // REQUIRES: rosNode is a pointer to the created node.
    // MODIFIES: n, subscribersByName, and publishersByName.
    // EFFECTS: Initializes all subscribers and publishers.
    static void init(ros::NodeHandle* rosNode);
};
