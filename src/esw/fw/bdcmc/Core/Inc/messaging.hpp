#pragma once

#include <array>
#include <variant>

#include "units.hpp"


namespace mrover {

    // CAN FD supports up to 64 bytes per frame
    constexpr size_t FRAME_SIZE = 64;

#define PACKED __attribute__((packed))

    using motor_id_t = uint8_t;

    struct BaseCommand {
        motor_id_t id;
    } PACKED;

    struct ConfigCommand : BaseCommand {
        Volts maxVolts;
    } PACKED;

    struct IdleCommand : BaseCommand {
    } PACKED;

    struct ThrottleCommand : BaseCommand {
        dimensionless_t throttle;
    } PACKED;

    struct VelocityCommand : BaseCommand {
        RadiansPerSecond velocity;
    } PACKED;

    struct PositionCommand : BaseCommand {
        Radians position;
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

} // namespace mrover
