#pragma once

#include "../can_library/can_manager.hpp"
#include <string>
#include <units/units.hpp>

enum class MotorType {
    Brushed = 0,
    Brushless = 1
};

class Controller {
public:
    Controller(ros::NodeHandle* n, std::string name) : name(name), can_manager(CANManager(n, name)) {}

    // will receive CAN frame with updated motor information (current speed, position, etc.)
    virtual void update(uint64_t frame) = 0;

    virtual void set_desired_speed_throttle(double velocity) = 0; // from -1.0 to 1.0
    virtual void set_desired_position(int position) = 0;
    virtual MotorType get_type() = 0;

    CANManager& get_can_manager() {
        return can_manager;
    }

protected:
    std::string name;
    CANManager can_manager;
    double velocity{};
    int position{};


    virtual void send_CAN_frame(uint64_t frame);
};
