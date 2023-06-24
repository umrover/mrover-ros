#pragma once

#include <array>
#include <variant>

#include "units.hpp"

// CAN FD supports up to 64 bytes per frame
constexpr size_t FRAME_SIZE = 64;

#define PACKED __attribute__((packed))

struct IdleCommand {
};

struct ThrottleCommand {
    double throttle;
} PACKED;

struct VelocityCommand {
    meters_per_second velocity;
} PACKED;

struct PositionCommand {
    meter position;
} PACKED;

struct StatusState {

} PACKED;

using Message = std::variant<
        IdleCommand, ThrottleCommand, VelocityCommand, PositionCommand>;
static_assert(sizeof(Message) <= FRAME_SIZE);

union FdCanFrame {
    Message message;
    std::array<std::byte, FRAME_SIZE> bytes;
};
