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

        // A1/A2 is 1 if pin connected to power, 0 if pin connected to ground
        AbsoluteEncoder(SMBus _i2cBus, uint8_t _A1, uint8_t _A2)
            : m_i2cBus(_i2cBus)
            , m_angle_rad(0)
            {
            // could be put into member list if we use ternary
            if (_A1 && _A2) {
                this->m_address = device_slave_address_both_power;
            } else if (_A1) {
                this->m_address = device_slave_address_a1_power;
            } else if (_A2) {
                this->m_address = device_slave_address_a2_power;
            } else { 
                this->m_address = device_slave_address_none_power;
            }
        }

        int read_raw_angle() {
            int raw_data = this->m_i2cBus.read_word_data(this->m_address, 0xFF);
            int angle_left = ( raw_data >> 8 ) & 0xFF; // 0xFE
            int angle_right = raw_data & 0xFF; // 0xFF
            int angle_left_modified = angle_left & 0x3F;
            int angle_raw = (angle_right << 6) | angle_left_modified;
            return angle_raw;
        }

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
