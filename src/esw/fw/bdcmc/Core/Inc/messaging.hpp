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
        Meters gear_ratio;
        // TODO: Terrible naming for the limit switch info
        uint8_t limit_switch_info_x;
        uint8_t limit_switch_info_y;
        uint8_t limit_switch_info_z;
        uint8_t quad_abs_enc_info;
        Meters   limit_a_readj_pos;
        Meters   limit_b_readj_pos;
        Meters   limit_c_readj_pos;
        Meters   limit_d_readj_pos;
        Meters   quad_enc_out_ratio;
        Meters   abs_enc_out_ratio;
        Dimensionless   max_pwm;
        uint8_t limit_max_pos;
        Meters   max_forward_pos;
        Meters   max_back_pos;
    };

    struct IdleCommand : BaseCommand {
    };

    struct ThrottleCommand : BaseCommand {
        Dimensionless throttle;
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
