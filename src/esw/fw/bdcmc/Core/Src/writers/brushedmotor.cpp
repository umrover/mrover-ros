#include "writer.hpp"

#include <config.hpp>
#include <hardware.hpp>

#include "main.h"

namespace mrover {

    BrushedMotorWriter::BrushedMotorWriter(TIM_HandleTypeDef* timer) {
        // Define Writer Constants
        this->timer = timer;
        this->channel = TIM_CHANNEL_1;
        this->arr = &TIM15->ARR;
        this->ccr = &TIM15->CCR1;

        // Direction pins
        this->forward_pin = Pin(GPIOC, GPIO_PIN_6);
        this->reverse_pin = Pin(GPIOB, GPIO_PIN_15);

        HAL_TIM_PWM_Start(this->timer, this->channel);
    }

    void BrushedMotorWriter::target_target_duty_cycle(Config const& config, Percent output) {
        // Set direction pins/duty cycle
        this->max_duty_cycle = config.get_max_duty_cyce();
        this->target_duty_cycle = output;
    }

    void BrushedMotorWriter::write() {
        this->set_direction_pins(this->target_duty_cycle);
        this->set_duty_cycle(this->target_duty_cycle);
    }

    void BrushedMotorWriter::set_direction_pins(Percent duty_cycle) {
        this->forward_pin.write(duty_cycle > 0_percent ? GPIO_PIN_SET : GPIO_PIN_RESET);
        this->reverse_pin.write(duty_cycle < 0_percent ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    void BrushedMotorWriter::set_duty_cycle(Percent duty_cycle) {
        // Clamp absolute value of the duty cycle to the supported range
        duty_cycle = std::clamp(abs(duty_cycle), 0_percent, this->max_duty_cycle);

        // Set CCR register
        // The CCR register compares its value to the timer and outputs a signal based on the result
        // The ARR register sets the limit for when the timer register resets to 0.
        if (this->ccr && this->arr) {
            auto limit = static_cast<float>(*this->arr);
            *this->ccr = static_cast<std::uint32_t>(std::round(duty_cycle.get() * limit));
        }
        // TODO(quintin) we should error if the registers are null pointers
    }

} // namespace mrover
