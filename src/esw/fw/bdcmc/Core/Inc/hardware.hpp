#pragma once

#include "main.h"

namespace mrover {

    class Pin {
    public:
        Pin(GPIO_TypeDef *port, uint16_t pin) : port(port), pin(pin) { }

        inline bool read() {
            return HAL_GPIO_ReadPin(this->port, this->pin);
        }

    private:
        GPIO_TypeDef *port;
        uint16_t pin;
    };

}
