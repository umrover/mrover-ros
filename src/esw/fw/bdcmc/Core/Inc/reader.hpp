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

        AbsoluteEncoder(SMBus _i2cBus, uint8_t _A1, uint8_t _A2);
        int read_raw_angle();

    private:
        int m_address;
        float m_angle_rad;
        SMBus m_i2cBus;
        enum {
            device_slave_address_none_power = 0x40,
            device_slave_address_a1_power = 0x41,
            device_slave_address_a2_power = 0x42,
            device_slave_address_both_power = 0x43,
        };
    };

    class QuadEncoder {
    public:
        QuadEncoder() = default;
        ~QuadEncoder() = default;

        void update();

    private:

    };


    class EncoderReader {
    public:
        EncoderReader() = default;
        [[nodiscard]] Radians read_input(const Config& config) const;

    private:
        void init(TIM_HandleTypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c);
        void refresh_absolute();
        uint32_t read_absolute();
        void update_count();
        TIM_HandleTypeDef* m_relative_encoder_timer{};
        I2C_HandleTypeDef* m_absolute_encoder_i2c{};
        Radians rotation{};
        Radians absolute_relative_diff{};
        AbsoluteEncoder abs_encoder;
    };

} // namespace mrover
