#pragma once

#include <numbers>
#include <cstdint>
#include <optional>

#include "hardware.hpp"
#include "hardware_i2c.hpp"
#include "units/units.hpp"

constexpr std::uint32_t COUNTS_PER_ROTATION_RELATIVE = 4096;
constexpr std::uint32_t COUNTS_PER_ROTATION_ABSOLUTE = 1024;
constexpr auto RADIANS_PER_COUNT_RELATIVE = mrover::Radians{2 * std::numbers::pi} / COUNTS_PER_ROTATION_RELATIVE;
constexpr auto RADIANS_PER_COUNT_ABSOLUTE = mrover::Radians{2 * std::numbers::pi} / COUNTS_PER_ROTATION_ABSOLUTE;
constexpr auto SECONDS_PER_TICK = mrover::Seconds{1 / 1000.0};

namespace mrover {

    struct EncoderReading {
        Radians position;
        RadiansPerSecond velocity;
    };

    class AbsoluteEncoderReader {
    public:
        AbsoluteEncoderReader() = default;

        AbsoluteEncoderReader(SMBus i2c_bus, std::uint8_t A1, std::uint8_t A2, Ratio multiplier);

        std::optional<std::uint64_t> read_raw_angle();

        [[nodiscard]] std::optional<EncoderReading> read();

    private:
        struct I2CAddress {
            constexpr static std::uint16_t
                    device_slave_address_none_high = 0x40,
                    device_slave_address_a1_high = 0x41,
                    device_slave_address_a2_high = 0x42,
                    device_slave_address_both_high = 0x43;
        };

        std::uint16_t m_address{};
        SMBus m_i2cBus;

        std::uint64_t m_previous_raw_data{};

        std::uint32_t m_ticks_prev{};
        Radians m_angle_prev;
        Ratio m_multiplier;

        Radians m_position;
        RadiansPerSecond m_velocity;
    };

    class QuadratureEncoderReader {
    public:
        QuadratureEncoderReader() = default;

        QuadratureEncoderReader(TIM_TypeDef* timer, Ratio multiplier);

        [[nodiscard]] std::optional<EncoderReading> read();

    private:
        TIM_TypeDef* m_timer{};
        std::int64_t m_counts_unwrapped_prev{};
        std::uint32_t m_counts_raw_now{};
        std::uint32_t m_ticks_prev{};
        std::uint32_t m_ticks_now{};
        Ratio m_multiplier;

        Radians m_position;
        RadiansPerSecond m_velocity;

        std::int64_t count_delta();
    };


    // obsolete?
    class FusedReader {
    public:
        FusedReader() = default;

        FusedReader(TIM_TypeDef* _tim, I2C_HandleTypeDef* absolute_encoder_i2c, Ratio quad_multiplier, Ratio abs_multiplier);

        [[nodiscard]] std::optional<EncoderReading> read();

    private:
        AbsoluteEncoderReader m_abs_encoder;
        QuadratureEncoderReader m_quad_encoder;

        TIM_TypeDef* m_relative_encoder_timer{};
        I2C_HandleTypeDef* m_absolute_encoder_i2c{};
        Radians m_position;
        RadiansPerSecond m_velocity;

        void refresh_absolute();
    };

    class LimitSwitchReader {};

} // namespace mrover
