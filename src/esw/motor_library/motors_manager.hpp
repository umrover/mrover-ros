#pragma once

#include "brushed.hpp"
#include "brushless.hpp"
#include "controller.hpp"
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <unordered_map>
#include <chrono>
#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>

class MotorsManager {
public:
    MotorsManager() = default;

    MotorsManager(ros::NodeHandle& n, const std::string& groupName, const std::vector<std::string>& controllerNames);

    Controller& get_controller(std::string const& name);

    void process_frame(int bus, int id, const std::vector<uint8_t> &frame_data);

    void moveMotorsThrottle(const mrover::Throttle::ConstPtr& msg);

    void moveMotorsVelocity(const mrover::Velocity::ConstPtr& msg);

    void moveMotorsPosition(const mrover::Position::ConstPtr& msg);

    void heartbeatCallback(const ros::TimerEvent&);

    void updateLastConnection();

private:
    std::unordered_map<std::string, std::unique_ptr<Controller>> controllers;
    std::unordered_map<int, std::string> names;
    std::string motorGroupName;
    std::vector<std::string> motorNames;
    std::chrono::high_resolution_clock::time_point lastConnection;

};