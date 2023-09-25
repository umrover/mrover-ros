#pragma once

#include <string>
#include <units/units.hpp>

enum class MotorType {
    Brushed = 0,
    Brushless = 1
};

class Controller {
public:
    // will receive CAN frame with updated motor information (current speed, position, etc.)
    virtual void update(uint64_t frame) = 0;

    virtual void set_desired_speed_unit(double velocity) = 0;  // from -1.0 to 1.0
    virtual void set_desired_position(int position) = 0;
    virtual MotorType get_type() = 0;

protected:
    std::string name{};
    int CAN_id{};
    double velocity{};
    int position{};
    ros::Publisher* CANPublisher;

    virtual void send_CAN_frame(uint64_t frame);
};
