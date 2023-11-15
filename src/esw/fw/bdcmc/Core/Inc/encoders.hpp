#pragma once

#include <cstdint>
#include <numbers>
#include <optional>

#include <hardware.hpp>
#include <hardware_i2c.hpp>
#include <units/units.hpp>

namespace mrover {

    constexpr auto tau = 2 * std::numbers::pi_v<float>;

    // Counts (ticks) per radian (NOT per rotation)
    using CPR = compound_unit<Ticks, inverse<Radians>>;

    constexpr auto RELATIVE_CPR = CPR{4096 / tau};
    constexpr auto ABSOLUTE_CPR = CPR{1024 / tau};

    struct EncoderReading {
        Radians position;
        RadiansPerSecond velocity;
    };

    class AbsoluteEncoderReader {
    public:
        AbsoluteEncoderReader() = default;

        AbsoluteEncoderReader(SMBus i2c_bus, std::uint8_t A1, std::uint8_t A2, Ratio multiplier);

        auto try_read_raw_angle() -> std::optional<std::uint64_t>;

        [[nodiscard]] auto read() -> std::optional<EncoderReading>;

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

        QuadratureEncoderReader(TIM_HandleTypeDef* timer, Ratio multiplier);

        [[nodiscard]] auto read() -> std::optional<EncoderReading>;

    private:
        TIM_HandleTypeDef* m_timer{};
        std::int64_t m_counts_unwrapped_prev{};
        std::uint32_t m_counts_raw_now{};
        std::uint32_t m_ticks_prev{};
        std::uint32_t m_ticks_now{};
        Ratio m_multiplier;

        Radians m_position;
        RadiansPerSecond m_velocity;

        std::int64_t count_delta();
    };

} // namespace mrover
