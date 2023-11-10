#pragma once

#include <array>
#include <cstdint>
#include <variant>

#include "units/units.hpp"

// Macros needed to operate on bitfields
#define SET_BIT_AT_INDEX(x, index, value) (x = (x & ~(1 << index)) | (value << index))
#define GET_BIT_AT_INDEX(x, index) (x & (1 << index))

namespace mrover {

#pragma pack(push, 1)

    struct ConfigLimitSwitchInfo {
        std::uint8_t present : 4 {};
        std::uint8_t enabled : 4 {};
        std::uint8_t active_high : 4 {};
        std::uint8_t limits_forward : 4 {};
        std::uint8_t is_default_enabled : 4 {};
        std::uint8_t limit_max_forward_position : 1 {};
        std::uint8_t limit_max_backward_position : 1 {};
        std::uint8_t use_for_readjustment : 4 {};
        std::array<Radians, 4> limit_readj_pos;
    };
    static_assert(sizeof(ConfigLimitSwitchInfo) == 20);

    struct ConfigEncoderInfo {
        [[maybe_unused]] std::uint8_t _ignore : 4 {}; // 8 bits - (4 meaningful bits) = 4 ignored bits
        std::uint8_t quad_present : 1 {};
        std::uint8_t quad_is_forward_polarity : 1 {};
        std::uint8_t abs_present : 1 {};
        std::uint8_t abs_is_forward_polarity : 1 {};
    };
    static_assert(sizeof(ConfigEncoderInfo) == 1);

    struct ConfigCalibErrorInfo {
        [[maybe_unused]] std::uint8_t _ignore : 2 {}; // 8 bits - (6 meaningful bits) = 2 ignored bits
        std::uint8_t configured : 1 {};
        std::uint8_t calibrated : 1 {};
        std::uint8_t error : 4 {}; // 0 means no error, anything else is error
    };
    static_assert(sizeof(ConfigCalibErrorInfo) == 1);

    struct LimitStateInfo {
        [[maybe_unused]] std::uint8_t _ignore : 4 {}; // 8 bits - (4 meaningful bits) = 4 ignored bits
        std::uint8_t hit : 4 {};
    };
    static_assert(sizeof(LimitStateInfo) == 1);

    struct BaseCommand {
    };

    struct AdjustCommand : BaseCommand {
        Radians position;
    };

    struct ConfigCommand : BaseCommand {
        Dimensionless gear_ratio;
        // TODO: Terrible naming for the limit switch info
        ConfigLimitSwitchInfo limit_switch_info;
        ConfigEncoderInfo quad_abs_enc_info;
        Ratio quad_enc_out_ratio;
        Ratio abs_enc_out_ratio;
        Percent max_pwm;
        Radians max_forward_pos;
        Radians max_backward_pos;
    };

    struct EnableLimitSwitchesCommand : BaseCommand {
        bool enable;
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

    struct ControllerDataState : BaseCommand {
        Radians position;
        RadiansPerSecond velocity;
        ConfigCalibErrorInfo config_calib_error_data;
        LimitStateInfo limit_switches;
    };

    using InBoundMessage = std::variant<
            AdjustCommand, ConfigCommand, EnableLimitSwitchesCommand, IdleCommand, ThrottleCommand, VelocityCommand, PositionCommand>;

    using OutBoundMessage = std::variant<
            ControllerDataState>;

    struct ArmLaserCommand : BaseCommand {
        bool enable;
    };

    struct LEDInfo {
        [[maybe_unused]] std::uint8_t _ignore : 4 {}; // 8 bits - (4 meaningful bits) = 4 ignored bits
        std::uint8_t red : 1 {};
        std::uint8_t green : 1 {};
        std::uint8_t blue : 1 {};
        std::uint8_t blinking : 1 {};
    };
    static_assert(sizeof(LEDInfo) == 1);

    struct LEDCommand : BaseCommand {
        LEDInfo led_info;
    };

    struct PDBData : BaseCommand {
        float temperature_24v{};
        float temperature_12v_jetson{};
        float temperature_12v_rest{};
        float temperature_12v_buck{};
        float temperature_5v{};
        float temperature_3v3{};
        float current_24v{};
        float current_12v_jetson{};
        float current_12v_rest{};
        float current_12v_buck{};
        float current_5v{};
        float current_3v3{};
    };

    using InBoundPDLBMessage = std::variant<
            ArmLaserCommand, LEDCommand>;

    using OutBoundPDLBMessage = std::variant<
            PDBData>;

    enum ScienceDevice {
        HEATER_B0,
        HEATER_N0,
        HEATER_B1,
        HEATER_N1,
        HEATER_B2,
        HEATER_N2,
    };

    struct EnableScienceDeviceCommand : BaseCommand {
        ScienceDevice science_device{};
        bool enable{};
    };

    struct HeaterAutoShutOffCommand : BaseCommand {
        bool enable_auto_shutoff{};
    };

    struct HeaterStateInfo {
        [[maybe_unused]] std::uint8_t _ignore : 2 {};
        std::uint8_t b0 : 1 {};
        std::uint8_t n0 : 1 {};
        std::uint8_t b1 : 1 {};
        std::uint8_t n1 : 1 {};
        std::uint8_t b2 : 1 {};
        std::uint8_t n2 : 1 {};
    };
    static_assert(sizeof(HeaterStateInfo) == 1);

    struct HeaterStateData : BaseCommand {
        HeaterStateInfo heater_state_info;
    };

    struct SpectralInfo {
        std::array<std::uint8_t, 6> data;
    };

    struct SpectralData : BaseCommand {
        std::array<SpectralInfo, 3> spectrals{};
    };

    struct ThermistorData : BaseCommand {
        float b0{};
        float n0{};
        float b1{};
        float n1{};
        float b2{};
        float n2{};
    };

    using InBoundScienceMessage = std::variant<
            EnableScienceDeviceCommand, HeaterAutoShutOffCommand>;

    using OutBoundScienceMessage = std::variant<
            HeaterStateData, SpectralData, ThermistorData>;


#pragma pack(pop)

} // namespace mrover
