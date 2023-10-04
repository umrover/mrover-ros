#pragma once

#include <numbers>
#include <units.hpp>
#include <config.hpp>

#include "stm32g4xx_hal.h"

constexpr uint32_t COUNTS_PER_ROTATION_RELATIVE = 4096;
constexpr uint32_t COUNTS_PER_ROTATION_ABSOLUTE = 1024;
constexpr auto RADIANS_PER_COUNT_RELATIVE = mrover::Radians{2 * std::numbers::pi} / COUNTS_PER_ROTATION_RELATIVE;
constexpr auto RADIANS_PER_COUNT_ABSOLUTE = mrover::Radians{2 * std::numbers::pi} / COUNTS_PER_ROTATION_ABSOLUTE;

namespace mrover {
    class AbsoluteEncoder {
    public:
        AbsoluteEncoder() = default;

        // A1/A2 is 1 if pin connected to power, 0 if pin connected to ground
        AbsoluteEncoder(SMBus i2cBus, uint8_t A1, uint8_t A2) {
            if (A1 && A2) {
                this->m_address = device_slave_address_both_power;
            } else if (A1) {
                this->m_address = device_slave_address_a1_power;
            } else if (A2) {
                this->m_address = device_slave_address_a2_power;
            } else { 
                this->m_address = device_slave_address_none_power;
            }
            this->m_i2cBus = i2cBus;
            this->m_angle_rad = 0;
        }

        long read_word_data(uint8_t addr, char cmd) {
            this->m_i2cBus->buf[0] = cmd;
            this->m_i2cBus->ret = HAL_I2C_Master_Transmit(this->m_i2cBus->i2c, addr << 1, this->m_i2cBus->buf, 1, 500);

            //reads from address sent above
            this->m_i2cBus->ret = HAL_I2C_Master_Receive(this->m_i2cBus->i2c, (addr << 1) | 1, this->m_i2cBus->buf, 2, 500);

            long data = this->m_i2cBus->buf[0] | (this->m_i2cBus->buf[1] << 8);
            if (this->m_i2cBus->ret != HAL_OK)
            {
                HAL_I2C_DeInit(this->m_i2cBus->i2c);
                HAL_Delay(5);
                HAL_I2C_Init(this->m_i2cBus->i2c);
                data = 0;
            }

            return data;
        }

        int read_raw_angle(AbsEncoder* abs_encoder) {
            int raw_data = read_word_data(abs_encoder->i2cBus, abs_encoder->address, 0xFF);
            int angle_left = ( raw_data >> 8 ) & 0xFF; // 0xFE
            int angle_right = raw_data & 0xFF; // 0xFF
            int angle_left_modified = angle_left & 0x3F;
            int angle_raw = (angle_right << 6) | angle_left_modified;
            return angle_raw;
        }


        void refresh_angle_radians() {
            int angle_raw = read_raw_angle(encoder);
            float radians = (float)angle_raw / RAW_TO_RADIANS_CONVERSION_FACTOR;
            encoder->angle_rad = radians;
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
    };

} // namespace mrover
