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
        LimitSwitch(uint8_t _valid, Pin _pin)
            : m_pin(_pin)
            , m_enabled(0)
            , m_is_pressed(0)
            , m_valid(_valid)
            , m_active_high(0)
            , m_associated_count(0)
            { }

        void update_limit_switch() {
            // This suggests active low
            if (this->m_valid && this->m_enabled) {
                this->m_is_pressed = (this->m_active_high == this->m_pin.read());
            }
            else {
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

void update_limit_switch(LimitSwitch *limit_switch) {
	// This suggests active low
	if (limit_switch->valid && limit_switch->enabled) {
		limit_switch->is_pressed = (limit_switch->active_high == read_pin_value(limit_switch->pin));
	}
	else {
		limit_switch->is_pressed = 0;
	}
}   };
}
