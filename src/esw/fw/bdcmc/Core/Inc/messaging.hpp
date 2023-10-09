#pragma once

#include <array>
#include <variant>

#include "units.hpp"


namespace mrover {

    // CAN FD supports up to 64 bytes per frame
    constexpr size_t FRAME_SIZE = 64;

    struct BaseCommand {
    };

    struct ConfigCommand : BaseCommand {
        Volts maxVolts;
    };

    struct IdleCommand : BaseCommand {
    };

    struct ThrottleCommand : BaseCommand {
        dimensionless_t throttle;
    };

    struct VelocityCommand : BaseCommand {
        RadiansPerSecond velocity;
    };

    struct PositionCommand : BaseCommand {
        Radians position;
    };

    struct StatusState {
    };

    using Message = std::variant<
            IdleCommand, ThrottleCommand, VelocityCommand, PositionCommand>;

    union FdCanFrame {
        Message message;
        std::array<std::byte, FRAME_SIZE> bytes;
    };
    static_assert(sizeof(FdCanFrame) == FRAME_SIZE);

} // namespace mrover
