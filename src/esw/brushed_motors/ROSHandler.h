#pragma once

#include "ControllerMap.h"          // for ControllerMap
#include <array>                    // for array
#include <cmath>                    // for nan
#include <mrover/Carousel.h>        // for Carousel
#include <mrover/MastGimbal.h>      // for MastGimbal
#include <optional>                 // for optional
#include <ros/console.h>            // for ROS_ERROR
#include <ros/ros.h>                // for ros
#include <sensor_msgs/JointState.h> // for JointState
#include <unordered_map>            // for unordered_map
#include <vector>                   // for vector

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

    // RA
    inline static std::array<std::string, 5> RANames;
    inline static ros::Subscriber moveRASubscriber;
    inline static ros::Publisher jointDataPublisherRA;
    inline static sensor_msgs::JointState jointDataRA;

    // SA
    inline static std::array<std::string, 5> SANames;
    inline static ros::Subscriber moveSASubscriber;
    inline static ros::Publisher jointDataPublisherSA;
    inline static sensor_msgs::JointState jointDataSA;

    // Cache
    inline static ros::Subscriber moveCacheSubscriber;

    // Carousel
    inline static ros::Subscriber moveCarouselSubscriber;

    // Mast
    inline static ros::Subscriber moveMastGimbalSubscriber;

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Moves a controller in open loop.
    static std::optional<float> moveControllerOpenLoop(const std::string& name, float velocity);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Moves a controller in closed loop.
    static std::optional<float> moveControllerClosedLoop(const std::string& name, float position);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Moves the RA joints in open loop and publishes angle data right after.
    static void moveRA(const sensor_msgs::JointState::ConstPtr& msg);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Moves the SA joints in open loop
    // and publishes angle data right after.
    static void moveSA(const sensor_msgs::JointState::ConstPtr& msg);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Moves the cache in open loop
    static void moveCache(const sensor_msgs::JointState::ConstPtr& msg);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Moves the carousel in either open loop or closed loop depending on the msg
    static void moveCarousel(const mrover::Carousel::ConstPtr& msg);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Moves a mast gimbal.
    static void moveMastGimbal(const mrover::MastGimbal::ConstPtr& msg);

public:
    // REQUIRES: rosNode is a pointer to the created node.
    // MODIFIES: static variables
    // EFFECTS: Initializes all subscribers and publishers.
    static void init(ros::NodeHandle* rosNode);
};
