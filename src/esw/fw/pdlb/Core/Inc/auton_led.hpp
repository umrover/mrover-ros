#pragma once

#include "stm32g4xx_hal.h"

#include "hardware.hpp"

namespace mrover {

    class AutonLed {
    public:
    	AutonLed() = default;

    	AutonLed(Pin red_pin, Pin green_pin, Pin blue_pin);

        void change_state(bool red, bool green, bool blue, bool blinking);

        void blink();

    private:
        Pin m_red_pin;
        Pin m_green_pin;
        Pin m_blue_pin;

        bool m_red{true};
        bool m_green{true};
        bool m_blue{true};
        bool m_blinking{true};
        bool m_on{};

        void change_all_pins();
    };

} // namespace mrover
