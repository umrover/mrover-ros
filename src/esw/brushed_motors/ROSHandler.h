#pragma once

#include "ControllerMap.h"          // for ControllerMap
#include <ros/ros.h>                // for ros
#include <sensor_msgs/JointState.h> // for JointState
#include <mrover/GimbalCmd.h>       // for GimbalCmd
#include <unordered_map>            // for unordered_map
#include <vector>                   // for vector
#include <cmath>                    // for nan

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
    inline static std::vector<std::string> RANames;
    inline static ros::Subscriber openLoopSubscriberRA;
    inline static ros::Subscriber openLoopSubscriberMast;

    inline static ros::Publisher jointDataPublisherRA;
    inline static sensor_msgs::JointState jointData;

    // REQUIRES: name is a valid name
    // MODIFIES: nothing
    // EFFECTS: Moves the RA joints in open loop
    // and publishes angle data right after.
    static void moveOpenLoopRACommand(const sensor_msgs::JointState::ConstPtr& msg);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Moves a gimbal.
    static void moveGimbal(const mrover::GimbalCmd::ConstPtr& msg);

public:
    // REQUIRES: rosNode is a pointer to the created node.
    // MODIFIES: static variables
    // EFFECTS: Initializes all subscribers and publishers.
    static void init(ros::NodeHandle* rosNode);
};
