#pragma once

#include <string>

#include "can_manager.hpp"
#include <units/units.hpp>
#include <ros/ros.h>

class Controller {
public:
    Controller(ros::NodeHandle& n, const std::string& name) : name{name}, can_manager{CANManager{n, name}} {}

    virtual ~Controller() = default;

    // will receive CAN frame with updated motor information (current speed, position, etc.)
    virtual void update(const std::vector<uint8_t> &frame) = 0;

    virtual void set_desired_throttle(float throttle) = 0; // from -1.0 to 1.0
    virtual void set_desired_velocity(float velocity) = 0; // in rad/s of joint output
    virtual void set_desired_position(float position) = 0; // in rad of joint

    CANManager& get_can_manager() {
        return can_manager;
    }

protected:
    std::string name;
    CANManager can_manager;
    float velocity{};
    float position{};
    float min_velocity{};  // this is min_velocity of joint output
    float max_velocity{};  // this is max_velocity of joint output
    float min_position{};  // this is min_position of joint output
    float max_position{};  // this is min_position of joint output

    //    virtual void send_CAN_frame(uint64_t frame) = 0;
};
