#pragma once

#include <array>
#include <cstdint>
#include <variant>

#include "hardware.hpp"
#include "units.hpp"

namespace mrover {

    struct __attribute__((__packed__)) ConfigLimitSwitchInfo0 {
        std::uint8_t a_present : 1;
        std::uint8_t b_present : 1;
        std::uint8_t c_present : 1;
        std::uint8_t d_present : 1;
        std::uint8_t a_enable : 1;
        std::uint8_t b_enable : 1;
        std::uint8_t c_enable : 1;
        std::uint8_t d_enable : 1;
    };
    static_assert(sizeof(ConfigLimitSwitchInfo0) == 1);

    struct __attribute__((__packed__)) ConfigLimitSwitchInfo1 {
        std::uint8_t a_active_high : 1;
        std::uint8_t b_active_high : 1;
        std::uint8_t c_active_high : 1;
        std::uint8_t d_active_high : 1;
        std::uint8_t a_limits_forward : 1;
        std::uint8_t b_limits_forward : 1;
        std::uint8_t c_limits_forward : 1;
        std::uint8_t d_limits_forward : 1;
    };
    static_assert(sizeof(ConfigLimitSwitchInfo1) == 1);

    struct __attribute__((__packed__)) ConfigLimitSwitchInfo2 {
        std::uint8_t a_use_for_readjustment : 1;
        std::uint8_t b_use_for_readjustment : 1;
        std::uint8_t c_use_for_readjustment : 1;
        std::uint8_t d_use_for_readjustment : 1;
        std::uint8_t a_is_default_enabled : 1;
        std::uint8_t b_is_default_enabled : 1;
        std::uint8_t c_is_default_enabled : 1;
        std::uint8_t d_is_default_enabled : 1;
    };
    static_assert(sizeof(ConfigLimitSwitchInfo2) == 1);

    struct __attribute__((__packed__)) ConfigEncoderInfo {
        [[maybe_unused]] std::uint8_t _ignore : 4; // 8 bits - (4 meaningful bits) = 4 ignored bits
        std::uint8_t quad_present : 1;
        std::uint8_t quad_is_forward_polarity : 1;
        std::uint8_t abs_present : 1;
        std::uint8_t abs_is_forward_polarity : 1;
    };
    static_assert(sizeof(ConfigEncoderInfo) == 1);

    struct __attribute__((__packed__)) ConfigLimitInfo {
        [[maybe_unused]] std::uint8_t _ignore : 6; // 8 bits - (2 meaningful bits) = 6 ignored bits
        std::uint8_t limit_max_forward_position : 1;
        std::uint8_t limit_max_backward_position : 1;
    };

    struct BaseCommand {
    };

    struct ConfigCommand : BaseCommand {
        Dimensionless gear_ratio;
        // TODO: Terrible naming for the limit switch info
        ConfigLimitSwitchInfo0 limit_switch_info_0;
        ConfigLimitSwitchInfo1 limit_switch_info_1;
        ConfigLimitSwitchInfo2 limit_switch_info_2;
        ConfigEncoderInfo quad_abs_enc_info;
        Radians limit_a_readj_pos;
        Radians limit_b_readj_pos;
        Radians limit_c_readj_pos;
        Radians limit_d_readj_pos;
        Dimensionless quad_enc_out_ratio;
        Dimensionless abs_enc_out_ratio;
        Dimensionless max_pwm;
        ConfigLimitInfo limit_max_pos;
        Meters max_forward_pos;
        Meters max_back_pos;
    };

    struct IdleCommand : BaseCommand {
    };

    struct ThrottleCommand : BaseCommand {
        Percent throttle;
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
        std::uint8_t config_calib_error_data;
        // TODO: Are these going to be left or right aligned.
        std::uint8_t limit_switches;
    };

    struct StatusState {
    };

    using InBoundMessage = std::variant<
            ConfigCommand, IdleCommand, ThrottleCommand, VelocityCommand, PositionCommand>;
    using OutBoundMessage = std::variant<
            MotorDataState, StatusState>;

    union FdCanFrameIn {
        InBoundMessage message;
        std::array<std::byte, CANFD_MAX_FRAME_SIZE> bytes;
    };
    static_assert(sizeof(FdCanFrameIn) == CANFD_MAX_FRAME_SIZE);

    union FdCanFrameOut {
        OutBoundMessage message;
        std::array<std::byte, CANFD_MAX_FRAME_SIZE> bytes;
    };
    static_assert(sizeof(FdCanFrameOut) == CANFD_MAX_FRAME_SIZE);

} // namespace mrover
