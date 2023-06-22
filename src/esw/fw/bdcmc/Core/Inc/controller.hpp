#pragma once

#include "messaging.hpp"
#include "pidf.hpp"
#include "units.hpp"

class Controller {
private:
    ControlMessage m_message;
    PIDF<radians, volts, milliseconds> m_position_controller;

public:
    void feed(ControlMessage const& message);

    void feed(OpenLoop const& message);

    void feed(VelocityControl const& message);

    void feed(PositionControl const& message);

    void step();
};
