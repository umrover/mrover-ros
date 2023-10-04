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
        SMBus(I2C_HandleTypeDef* hi2c) : i2c(hi2c){
            for (size_t i = 0; i < 30; i++) {
                buf[i] = 0;
            }
        }

        long read_word_data(uint8_t addr, char cmd) {
            buf[0] = cmd;
            ret = HAL_I2C_Master_Transmit(i2c, addr << 1, buf, 1, 500);

            //reads from address sent above
            ret = HAL_I2C_Master_Receive(i2c, (addr << 1) | 1, buf, 2, 500);

            long data = buf[0] | (buf[1] << 8);
            if (ret != HAL_OK)
            {
                HAL_I2C_DeInit(i2c);
                HAL_Delay(5);
                HAL_I2C_Init(i2c);
                data = 0;
            }

            return data;
        }

    private:
        I2C_HandleTypeDef *i2c;
        HAL_StatusTypeDef ret;
        uint8_t buf[30];
        uint8_t DMA;
    };

}
