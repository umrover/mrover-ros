#pragma once

#include <config.hpp>
#include <hardware.hpp>
#include <numbers>
#include <units.hpp>

#include "main.h"

constexpr std::uint32_t COUNTS_PER_ROTATION_RELATIVE = 4096;
constexpr std::uint32_t COUNTS_PER_ROTATION_ABSOLUTE = 1024;
constexpr auto RADIANS_PER_COUNT_RELATIVE = mrover::Radians{2 * std::numbers::pi} / COUNTS_PER_ROTATION_RELATIVE;
constexpr auto RADIANS_PER_COUNT_ABSOLUTE = mrover::Radians{2 * std::numbers::pi} / COUNTS_PER_ROTATION_ABSOLUTE;


namespace mrover {

    class AbsoluteEncoder {
    public:
        AbsoluteEncoder() = default;
        AbsoluteEncoder(SMBus i2c_bus, std::uint8_t A1, std::uint8_t A2);

        std::optional<std::uint64_t> read_raw_angle();

    private:
        std::uint8_t m_address{};
        SMBus m_i2cBus;
        enum {
            // default i2c address is 0x40
            device_slave_address_none_high = 0x40,
            device_slave_address_a1_high = 0x41,
            device_slave_address_a2_high = 0x42,
            device_slave_address_both_high = 0x43,
        };

        std::uint64_t m_previous_raw_data{};
    };


    class QuadratureEncoder {
    public:
        QuadratureEncoder() = default;
        QuadratureEncoder(TIM_TypeDef* _tim);

        std::int64_t count_delta();

    private:
        TIM_TypeDef* m_timer{};
        std::uint32_t m_counts_raw_prev{};
        std::uint32_t m_counts_raw_now{};
    };


    class FusedReader {
    public:
        FusedReader() = default;
        FusedReader(TIM_HandleTypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c);
        [[nodiscard]] std::pair<Radians, RadiansPerSecond> read(Config const& config);

    private:
        AbsoluteEncoder m_abs_encoder;
        QuadratureEncoder m_quad_encoder;

        TIM_HandleTypeDef* m_relative_encoder_timer{};
        I2C_HandleTypeDef* m_absolute_encoder_i2c{};
        Radians m_position{};
        RadiansPerSecond m_velocity{};

        void refresh_absolute();
    };

} // namespace mrover
