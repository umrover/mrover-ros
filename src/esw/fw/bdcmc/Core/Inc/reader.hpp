#pragma once

#include <numbers>
#include <units.hpp>
#include <config.hpp>
#include <hardware.hpp>

#include "main.h"


constexpr uint32_t COUNTS_PER_ROTATION_RELATIVE = 4096;
constexpr uint32_t COUNTS_PER_ROTATION_ABSOLUTE = 1024;
constexpr auto RADIANS_PER_COUNT_RELATIVE = mrover::Radians{2 * std::numbers::pi} / COUNTS_PER_ROTATION_RELATIVE;
constexpr auto RADIANS_PER_COUNT_ABSOLUTE = mrover::Radians{2 * std::numbers::pi} / COUNTS_PER_ROTATION_ABSOLUTE;


namespace mrover {

    class AbsoluteEncoder {
    public:
        AbsoluteEncoder() = default;
        AbsoluteEncoder(SMBus i2cBus, uint8_t A1, uint8_t A2);
        uint64_t read_raw_angle();
    private:
        uint32_t m_address{};
        SMBus m_i2cBus{};
        enum {
            // default i2c address is 0x40
            device_slave_address_none_high = 0x40,
            device_slave_address_a1_high = 0x41,
            device_slave_address_a2_high = 0x42,
            device_slave_address_both_high = 0x43,
        };
    };


    class QuadEncoder {
    public:
        QuadEncoder() = default;
        QuadEncoder(TIM_TypeDef *_tim);
        ~QuadEncoder() = default;
        int64_t count_delta();
    private:
        TIM_TypeDef* m_tim;
        uint32_t m_counts_raw_prev;
        uint32_t m_counts_raw_now;
    };


    class EncoderReader {
    public:
        EncoderReader() = default;
        EncoderReader(TIM_HandleTypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c);
        [[nodiscard]] std::pair<Radians, RadiansPerSecond> read();
        void update(const Config& config);

    private:
        void refresh_absolute();
        uint64_t read_absolute();

        AbsoluteEncoder m_abs_encoder;
        QuadEncoder m_quad_encoder{};

        TIM_HandleTypeDef* m_relative_encoder_timer{};
        I2C_HandleTypeDef* m_absolute_encoder_i2c{};
        Radians position{};
        RadiansPerSecond velocity{};
    };

} // namespace mrover
