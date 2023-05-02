#pragma once
#include "ControllerMap.h"          // for ControllerMap
#include <algorithm>                // for distance
#include <array>                    // for array
#include <cmath>                    // for nan
#include <mrover/AdjustMotors.h>    // for AdjustMotors
#include <mrover/CalibrateMotors.h> // for CalibrateMotors
#include <mrover/Calibrated.h>      // for Calibrated
#include <mrover/Carousel.h>        // for Carousel
#include <mrover/EnableDevice.h>    // for EnableDevice
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

    // Calibrate service
    inline static ros::ServiceServer calibrateService;
    inline static ros::ServiceServer adjustService;
    inline static ros::ServiceServer adjustUsingAbsEncService;
    inline static ros::ServiceServer enableLimitSwitchService;

    // RA
    inline static std::array<std::string, 5> RANames;
    inline static ros::Subscriber moveRASubscriber;
    inline static ros::Publisher jointDataPublisherRA;
    inline static sensor_msgs::JointState jointDataRA;
    inline static ros::Publisher calibrationStatusPublisherRA;
    inline static mrover::Calibrated calibrationStatusRA;

    // SA
    inline static std::array<std::string, 5> SANames;
    inline static ros::Subscriber moveSASubscriber;
    inline static ros::Publisher jointDataPublisherSA;
    inline static sensor_msgs::JointState jointDataSA;
    inline static ros::Publisher calibrationStatusPublisherSA;
    inline static mrover::Calibrated calibrationStatusSA;

    // Cache
    inline static ros::Subscriber moveCacheSubscriber;

    // Carousel
    inline static std::string carousel_name;
    inline static ros::Subscriber moveCarouselSubscriber;
    inline static ros::Publisher calibrationStatusPublisherCarousel;
    inline static mrover::Calibrated calibrationStatusCarousel;

    // Mast
    inline static ros::Subscriber moveMastGimbalSubscriber;

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Moves a controller in open loop.
    static std::optional<float> moveControllerOpenLoop(const std::string& name, float velocity);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: Determine if a controller is calibrated
    static std::optional<bool> getControllerCalibrated(const std::string& name);

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

    // REQUIRES: valid req and res objects
    // MODIFIES: res
    // EFFECTS: sends a move/calibration command to the mcu
    static bool processMotorCalibrate(mrover::CalibrateMotors::Request& req, mrover::CalibrateMotors::Response& res);

    // REQUIRES: valid req and res objects
    // MODIFIES: res
    // EFFECTS: hard sets the requested controller angle
    static bool processMotorAdjust(mrover::AdjustMotors::Request& req, mrover::AdjustMotors::Response& res);

    // REQUIRES: valid req and res objects
    // MODIFIES: res
    // EFFECTS: takes the current absolute encoder value, applies an offset, and hard sets the new angle
    static bool processMotorAdjustUsingAbsEnc(mrover::AdjustMotors::Request& req, mrover::AdjustMotors::Response& res);

    // REQUIRES: valid req and res objects
    // MODIFIES: res
    // EFFECTS: disables or enables limit switches
    static bool processMotorEnableLimitSwitches(mrover::EnableDevice::Request& req, mrover::EnableDevice::Response& res);

    // REQUIRES: mcu_id is a valid mcu_id
    // MODIFIES: nothing
    // EFFECTS: resets the tick of a watchdog for a particular mcu using the On function
    static void tickMCU(int mcu_id);

    // REQUIRES: mcu_id is a valid mcu_id
    // MODIFIES: nothing
    // EFFECTS: resets the tick of a watchdog for all MCUs
    static void tickAllMCUs();

public:
    // REQUIRES: rosNode is a pointer to the created node.
    // MODIFIES: static variables
    // EFFECTS: Initializes all subscribers and publishers.
    static void init(ros::NodeHandle* rosNode);

    // REQUIRES: name is the name of a controller and isCalibrated is whether it is calibrated
    // MODIFIES: static variables
    // EFFECTS: Publishes calibration status to the proper topic depending on the name
    static void publishCalibrationDataUsingName(const std::string& name, bool isCalibrated);

    // REQUIRES: nothing
    // MODIFIES: nothing
    // EFFECTS: used as a watchdog for the MCUs
    static void timerCallback(const ros::TimerEvent &event);
};
