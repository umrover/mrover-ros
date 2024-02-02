#include "hbridge.hpp"

#include <algorithm>
#include <cmath>

namespace mrover {

    HBridge::HBridge(TIM_HandleTypeDef* timer, Pin forward_pin, Pin reverse_pin)
        : m_forward_pins{forward_pin},
          m_reverse_pins{reverse_pin},
          m_timer{timer},
          m_arr_register{&TIM15->ARR},
          m_ccr_register{&TIM15->CCR1},
          m_max_pwm{0_percent} {

        // Prevent the motor from spinning on boot up
        *m_ccr_register = 0;
        HAL_TIM_PWM_Start(m_timer, m_channel);
    }

    void HBridge::write(Percent output) const {
        // Set direction pins/duty cycle
        set_direction_pins(output);
        set_duty_cycle(output, m_max_pwm);
    }

    void HBridge::set_direction_pins(Percent duty_cycle) const {
        m_forward_pins.write(duty_cycle < 0_percent ? GPIO_PIN_SET : GPIO_PIN_RESET);
        m_reverse_pins.write(duty_cycle > 0_percent ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    void HBridge::set_duty_cycle(Percent duty_cycle, Percent max_duty_cycle) const {
        // Clamp absolute value of the duty cycle to the supported range
        duty_cycle = std::clamp(abs(duty_cycle), 0_percent, max_duty_cycle);

        // Set CCR register
        // The CCR register compares its value to the timer and outputs a signal based on the result
        // The ARR register sets the limit for when the timer register resets to 0.
        if (m_ccr_register && m_arr_register) {
            auto limit = static_cast<float>(*m_arr_register);
            *m_ccr_register = static_cast<std::uint32_t>(std::round(duty_cycle.get() * limit));
        }
        // TODO(quintin) we should error if the registers are null pointers
    }

    void HBridge::change_max_pwm(Percent max_pwm) {
        m_max_pwm = max_pwm;
    }

} // namespace mrover
