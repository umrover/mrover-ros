#pragma once

#include <cstdint>
#include <numbers>
#include <optional>

#include <hardware.hpp>
#include <hardware_i2c.hpp>
#include <units/units.hpp>

#include "common.hpp"
#include "filtering.hpp"

namespace mrover {

    struct EncoderReading {
        Radians position;
        RadiansPerSecond velocity;
    };

    class AbsoluteEncoderReader {
    public:
        using AS5048B_Bus = SMBus<std::uint8_t, std::array<std::uint8_t, 2>>;

        AbsoluteEncoderReader() = default;

        AbsoluteEncoderReader(AS5048B_Bus i2c_bus, Radians offset, Ratio multiplier, TIM_HandleTypeDef* elapsed_timer);

        auto request_raw_angle() -> void;
        auto read_raw_angle_into_buffer() -> void;
        auto try_read_buffer() -> std::optional<std::uint64_t>;

        [[nodiscard]] auto read() -> std::optional<EncoderReading>;

    private:
        struct I2CAddress {
            constexpr static std::uint16_t
                    device_slave_address_none_high = 0x40,
                    device_slave_address_a1_high = 0x41,
                    device_slave_address_a2_high = 0x42,
                    device_slave_address_both_high = 0x43;
        };

        TIM_HandleTypeDef* m_elapsed_timer{};

        std::uint16_t m_address = I2CAddress::device_slave_address_none_high;
        AS5048B_Bus m_i2cBus;

        std::uint64_t m_previous_raw_data{};

        Radians m_offset;
        Ratio m_multiplier;

        Radians m_position;
        Radians m_position_prev;
        RunningMeanFilter<RadiansPerSecond, 16> m_velocity_filter;
    };

    class QuadratureEncoderReader {
    public:
        QuadratureEncoderReader() = default;

        QuadratureEncoderReader(TIM_HandleTypeDef* tick_timer, Ratio multiplier, TIM_HandleTypeDef* elapsed_timer);

        [[nodiscard]] auto read() const -> std::optional<EncoderReading>;

        auto update() -> void;

        auto expired() -> void {
            m_velocity_filter.clear();
        }

    private:
        TIM_HandleTypeDef* m_tick_timer{};
        TIM_HandleTypeDef* m_elapsed_timer{};
        std::int64_t m_counts_unwrapped_prev{};
        Ratio m_multiplier;

        Radians m_position;
        // RadiansPerSecond m_velocity;
        RunningMeanFilter<RadiansPerSecond, 16> m_velocity_filter;
        // IIRFilter<RadiansPerSecond, 0.05> m_velocity_filter;
    };

} // namespace mrover