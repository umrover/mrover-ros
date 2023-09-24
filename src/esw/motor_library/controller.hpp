#pragma once

#include <string>
#include <units/units.hpp>

class Controller {
public:
    // will receive CAN frame with updated motor information (current speed, position, etc.)
    virtual void update(uint64_t frame) = 0;

    virtual void set_desired_speed(double velocity) = 0;
    virtual void set_desired_position(int position) = 0;

protected:
    std::string name{};
    int CAN_id{};
    double velocity{};
    int position{};

    virtual void send_CAN_frame(uint64_t frame);
};
