#pragma once

#include <string>

#include <can_manager.hpp>
#include <units/units.hpp>

#include <ros/ros.h>

enum class MotorType {
    Brushed = 0,
    Brushless = 1
};

class Controller {
public:
    Controller(ros::NodeHandle& n, const std::string& name) : name{name}, can_manager{CANManager{n, name}} {}

    virtual ~Controller() = default;

    // will receive CAN frame with updated motor information (current speed, position, etc.)
    virtual void update(uint64_t frame) = 0;

    virtual void set_desired_throttle(double throttle) = 0; // from -1.0 to 1.0
    virtual void set_desired_velocity(double velocity) = 0; // in rad/s
    virtual void set_desired_position(double position) = 0; // in rad
    virtual MotorType get_type() = 0;

    CANManager& get_can_manager() {
        return can_manager;
    }

protected:
    std::string name;
    CANManager can_manager;
    double velocity{};
    int position{};

    //    virtual void send_CAN_frame(uint64_t frame) = 0;
};
