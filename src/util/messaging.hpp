#pragma once

#include <array>
#include <cstdint>
#include <variant>

#include <units/units.hpp>

namespace mrover {

#pragma pack(push, 1)

    struct ConfigLimitSwitchInfo {
        std::uint8_t present : 4 {};
        std::uint8_t enabled : 4 {};
        std::uint8_t active_high : 4 {};
        std::uint8_t limits_forward : 4 {};
        std::uint8_t limit_max_forward_position : 1 {};
        std::uint8_t limit_max_backward_position : 1 {};
        std::uint8_t use_for_readjustment : 4 {};
        std::array<Radians, 4> limit_readj_pos;
    };

    struct ConfigEncoderInfo {
        [[maybe_unused]] std::uint8_t _ignore : 4 {}; // 8 bits - (4 meaningful bits) = 4 ignored bits
        std::uint8_t quad_present : 1 {};
        std::uint8_t _quad_is_forward_polarity : 1 {};
        std::uint8_t abs_present : 1 {};
        std::uint8_t _abs_is_forward_polarity : 1 {};
        Ratio quad_ratio;
        Ratio abs_ratio;
        Radians abs_offset;
    };

    enum struct BDCMCErrorInfo : std::uint8_t {
        NO_ERROR,
        DEFAULT_START_UP_NOT_CONFIGURED,
        RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED,
        RECEIVING_POSITION_COMMANDS_WHEN_NOT_CALIBRATED,
        OUTPUT_SET_TO_ZERO_SINCE_EXCEEDING_LIMITS,
        RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS
    };

    struct ConfigCalibErrorInfo {
        [[maybe_unused]] std::uint8_t _ignore : 2 {}; // 8 bits - (6 meaningful bits) = 2 ignored bits
        std::uint8_t configured : 1 {};
        std::uint8_t calibrated : 1 {};
        BDCMCErrorInfo error : 4 {}; // 0 means no error, anything else is error
    };

    struct LimitStateInfo {
        [[maybe_unused]] std::uint8_t _ignore : 4 {}; // 8 bits - (4 meaningful bits) = 4 ignored bits
        std::uint8_t hit : 4 {};
    };

    struct BaseCommand {
    };

    struct AdjustCommand : BaseCommand {
        Radians position;
    };

    struct ConfigCommand : BaseCommand {
        Dimensionless gear_ratio;
        ConfigLimitSwitchInfo limit_switch_info;
        ConfigEncoderInfo enc_info;
        Percent max_pwm;
        [[maybe_unused]] std::uint8_t _ignore : 7 {};
        std::uint8_t is_inverted : 1 {};
        Radians min_position, max_position;
        RadiansPerSecond min_velocity, max_velocity;
    };

    struct IdleCommand : BaseCommand {
    };

    struct ThrottleCommand : BaseCommand {
        Percent throttle;
    };

    struct VelocityCommand : BaseCommand {
        RadiansPerSecond velocity;
        float p{}, i{}, d{}, ff{};
    };

    struct PositionCommand : BaseCommand {
        Radians position;
        float p{}, i{}, d{};
    };

    struct ControllerDataState {
        Radians position;
        RadiansPerSecond velocity;
        ConfigCalibErrorInfo config_calib_error_data;
        LimitStateInfo limit_switches;
    };

    struct DebugState {
        float f1{};
        float f2{};
        float f3{};
        float f4{};
    };

    using InBoundMessage = std::variant<
            AdjustCommand, ConfigCommand, IdleCommand, ThrottleCommand, VelocityCommand, PositionCommand>;

    using OutBoundMessage = std::variant<
            ControllerDataState, DebugState>;

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

    struct LEDCommand : BaseCommand {
        LEDInfo led_info;
    };

    struct PDBData : BaseCommand {
        // order is always 24V, then 12v Jetson, 12v rest, 12v buck, 5v, 3v3
        std::array<float, 6> temperatures{};
        std::array<float, 6> currents{};
    };

    using InBoundPDLBMessage = std::variant<
            ArmLaserCommand, LEDCommand>;

    using OutBoundPDLBMessage = std::variant<
            PDBData>;

    enum struct ScienceDevice {
        HEATER_B0,
        HEATER_N0,
        HEATER_B1,
        HEATER_N1,
        HEATER_B2,
        HEATER_N2,
        WHITE_LED_0,
        WHITE_LED_1,
        WHITE_LED_2,
        UV_LED_0,
        UV_LED_1,
        UV_LED_2
    };

    struct EnableScienceDeviceCommand : BaseCommand {
        ScienceDevice science_device{};
        bool enable{};
    };

    struct HeaterAutoShutOffCommand : BaseCommand {
        bool enable_auto_shutoff{};
    };

    struct ConfigThermistorAutoShutOffCommand : BaseCommand {
        float shutoff_temp{};
    };

    struct HeaterStateInfo {
        [[maybe_unused]] std::uint8_t _ignore : 2 {};
        std::uint8_t on : 6 {};
    };

    struct HeaterStateData : BaseCommand {
        HeaterStateInfo heater_state_info;
    };

    struct SpectralInfo {
        std::array<float, 6> data{};
        bool error{};
    };

    struct SpectralData : BaseCommand {
        std::array<float, 6> data{};
        std::uint8_t site;
        bool error{};
    };

    struct ThermistorData : BaseCommand {
        std::array<float, 6> temps{};
    };

    using InBoundScienceMessage = std::variant<
            EnableScienceDeviceCommand, HeaterAutoShutOffCommand, ConfigThermistorAutoShutOffCommand>;

    using OutBoundScienceMessage = std::variant<
            HeaterStateData, SpectralData, ThermistorData>;

#pragma pack(pop)

    // Utility for std::visit with lambdas
    template<class... Ts>
    struct overloaded : Ts... {
        using Ts::operator()...;
    };

} // namespace mrover

// Macros needed to operate on bitfields
#define SET_BIT_AT_INDEX(x, index, value) (x = (x & ~(1 << index)) | (value << index))
#define GET_BIT_AT_INDEX(x, index) (x & (1 << index))
