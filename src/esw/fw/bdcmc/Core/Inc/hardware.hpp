#pragma once

#include "main.h"

namespace mrover {

    class Pin {
    public:
        explicit Pin() = default;
        Pin(GPIO_TypeDef *port, uint16_t pin) : port(port), pin(pin) { }

        inline GPIO_PinState read() {
            return HAL_GPIO_ReadPin(this->port, this->pin);
        }

        inline void write(GPIO_PinState val) {
            HAL_GPIO_WritePin(this->port, this->pin, val);
        }

    private:
        GPIO_TypeDef *port{};
        uint16_t pin{};
    };

    class LimitSwitch {
    public:
        LimitSwitch(Pin _pin)
            : m_pin(_pin)
            , m_enabled(0)
            , m_is_pressed(0)
            , m_active_high(0)
            , m_associated_count(0)
            { }

        void update_limit_switch() {
            // This suggests active low
            if (this->m_enabled) {
                this->m_is_pressed = (this->m_active_high == this->m_pin.read());
            } else {
                this->m_is_pressed = 0;
            }
        }

    private:
        Pin m_pin;
        uint8_t m_enabled;
        uint8_t m_is_pressed;
        uint8_t m_valid;
        uint8_t m_active_high;
        int32_t m_associated_count;

    };

    class SMBus {
    public:
        SMBus() = default;

        explicit SMBus(I2C_HandleTypeDef* hi2c)
            : m_i2c(hi2c) {
            for (unsigned char & i : m_buf) {
                i = 0;
            }
        }

        uint64_t read_word_data(uint8_t addr, char cmd) {
            this->m_buf[0] = cmd;
            this->m_ret = HAL_I2C_Master_Transmit(this->m_i2c, addr << 1, this->m_buf, 1, 500);

            //reads from address sent above
            this->m_ret = HAL_I2C_Master_Receive(this->m_i2c, (addr << 1) | 1, this->m_buf, 2, 500);

            long data = this->m_buf[0] | (this->m_buf[1] << 8);
            if (this->m_ret != HAL_OK) {
                HAL_I2C_DeInit(this->m_i2c);
                HAL_Delay(5);
                HAL_I2C_Init(this->m_i2c);
                data = 0;
            }

            return data;
        }

    private:
        I2C_HandleTypeDef *m_i2c{};
        HAL_StatusTypeDef m_ret{};
        uint8_t m_buf[30]{};
    };

}
