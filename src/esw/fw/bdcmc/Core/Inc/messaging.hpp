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
    
    struct MotorDataState : BaseCommand {
        RadiansPerSecond velocity;
        Radians position;
        uint8_t config_calib_error_data;
        // TODO: Are these going to be left or right aligned.
        uint8_t limit_switches; 
    };

    struct StatusState {
    };

    using InBoundMessage = std::variant<
            ConfigCommand, IdleCommand, ThrottleCommand, VelocityCommand, PositionCommand>;
    using OutBoundMessage = std::variant<
            MotorDataState, StatusState>;

    union FdCanFrameIn {
        InBoundMessage message;
        std::array<std::byte, FRAME_SIZE> bytes;
    };
    static_assert(sizeof(FdCanFrameIn) == FRAME_SIZE);

    union FdCanFrameOut {
        OutBoundMessage message;
        std::array<std::byte, FRAME_SIZE> bytes;
    };
    static_assert(sizeof(FdCanFrameOut) == FRAME_SIZE);

} // namespace mrover
