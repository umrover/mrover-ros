#include "writer.hpp"

#include <config.hpp>
#include <hardware.hpp>

#include "main.h"

namespace mrover {

    HBridgeWriter::HBridgeWriter(TIM_HandleTypeDef* timer)
        : m_timer{timer},
          m_channel{TIM_CHANNEL_1},
          m_arr_register{&TIM15->ARR}, m_ccr_register{&TIM15->CCR1},
          m_forward_pins{GPIOC, GPIO_PIN_6}, m_reverse_pins{GPIOB, GPIO_PIN_13} {

        HAL_TIM_PWM_Start(m_timer, m_channel);
    }

    void HBridgeWriter::write(Config const& config, Percent output) {
        // Set direction pins/duty cycle
        set_direction_pins(output);
        set_duty_cycle(output, config.get_max_duty_cycle());
    }

    void HBridgeWriter::set_direction_pins(Percent duty_cycle) {
        m_forward_pins.write(duty_cycle > 0_percent ? GPIO_PIN_SET : GPIO_PIN_RESET);
        m_reverse_pins.write(duty_cycle < 0_percent ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    void HBridgeWriter::set_duty_cycle(Percent duty_cycle, Percent max_duty_cycle) {
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

} // namespace mrover
