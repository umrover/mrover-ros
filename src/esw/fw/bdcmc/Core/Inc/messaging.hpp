#pragma once

#include <array>
#include <variant>

#include "units.hpp"

constexpr size_t FRAME_SIZE = 64;

struct Idle {
};

struct OpenLoop {
    double throttle;
};

struct VelocityControl {
    meters_per_second velocity;
};

struct PositionControl {
    meters position;
};

using ControlMessage = std::variant<Idle, OpenLoop, VelocityControl, PositionControl>;
static_assert(sizeof(ControlMessage) <= FRAME_SIZE);

union FdCanFrame {
    ControlMessage message;
    std::array<std::byte, FRAME_SIZE> bytes;
};
