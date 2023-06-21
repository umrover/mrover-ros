#pragma once

#include "main.h"
#include "messaging.hpp"

class Controller {
private:
    ControlMessage m_message;

public:
    void feed(ControlMessage const& message);

    void feed(OpenLoop const& message);

    void feed(VelocityControl const& message);

    void feed(PositionControl const& message);

    void step();
};
