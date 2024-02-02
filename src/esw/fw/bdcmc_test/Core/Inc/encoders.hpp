#pragma once

#include <cstdint>
#include <numbers>
#include <optional>

#include <hardware.hpp>
#include <hardware_i2c.hpp>
#include <units/units.hpp>

#include "filtering.hpp"

namespace mrover {

    constexpr auto tau = 2 * std::numbers::pi_v<float>;

    // Counts (ticks) per radian (NOT per rotation)
    using CountsPerRad = compound_unit<Ticks, inverse<Radians>>;

    constexpr auto RELATIVE_CPR = CountsPerRad{3355 / tau}; // Measured empirically
    constexpr auto ABSOLUTE_CPR = CountsPerRad{1024 / tau};
    constexpr auto MIN_MEASURABLE_VELOCITY = RadiansPerSecond{0.05}; // Very thoroughly obtained number - Quintin Approves
    auto const CLOCK_FREQ = Hertz{HAL_RCC_GetHCLKFreq()};

    struct EncoderReading {
        Radians position;
        RadiansPerSecond velocity;
    };

    class AbsoluteEncoderReader {
    public:
        AbsoluteEncoderReader() = default;

        AbsoluteEncoderReader(SMBus<uint8_t, uint16_t> i2c_bus, std::uint8_t A1, std::uint8_t A2, Ratio multiplier, TIM_HandleTypeDef* elapsed_timer);

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

        std::uint16_t m_address{};
        SMBus<uint8_t, uint16_t> m_i2cBus;

        std::uint64_t m_previous_raw_data{};

        Ratio m_multiplier;

        Radians m_position;
        Radians m_position_prev;
        RadiansPerSecond m_velocity;
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
        // std::int64_t m_vel_counts_unwrapped_prev{};
        // std::uint32_t m_counts_raw_now{};
        // std::uint32_t m_ticks_prev{};
        // std::uint32_t m_ticks_now{};
        Ratio m_multiplier;
        // Seconds m_velocity_dt;

        Radians m_position;
        // RadiansPerSecond m_velocity;
        RunningMeanFilter<RadiansPerSecond, 4 + 1> m_velocity_filter;
    };

} // namespace mrover
