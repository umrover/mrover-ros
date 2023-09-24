#pragma once

#include <string>
#include <units/units.hpp>

class Controller {
public:
    virtual void set_velocity(float velocity) = 0;
    virtual void set_position(float speed, float position) = 0;

protected:
    std::string name{};
    float velocity{};
    float position{};
};
