#include "writer.hpp"

#include <config.hpp>
#include <hardware.hpp>

#include "main.h"


namespace mrover {

    BrushedMotorWriter::BrushedMotorWriter(TIM_HandleTypeDef* timer) {
        // Define Writer Constants
        this->timer = timer;
        this->channel = TIM_CHANNEL_1;
        this->arr = &(TIM15->ARR);
        this->ccr = &(TIM15->CCR1);

        // Direction pins
        this->forward_pin = Pin(GPIOC, GPIO_PIN_6);
        this->reverse_pin = Pin(GPIOB, GPIO_PIN_15);

        HAL_TIM_PWM_Start(this->timer, this->channel);
    }

    void BrushedMotorWriter::set_tgt(const Config& config, Dimensionless output) {
        // Set direction pins/duty cycle
        this->max_pwm = config.getMaxPWM();
        this->tgt_duty_cycle = output.get();
    }

    void BrushedMotorWriter::write() {
        this->set_direction_pins(this->tgt_duty_cycle);
        this->set_pwm(this->tgt_duty_cycle);
    }

    void BrushedMotorWriter::set_direction_pins(double duty_cycle) {
        this->forward_pin.write((duty_cycle > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        this->reverse_pin.write((duty_cycle < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    void BrushedMotorWriter::set_pwm(double duty_cycle) {
        // Obtain magnitude of duty_cycle and normalize
        duty_cycle *= duty_cycle > 0 ? 1 : -1;
        duty_cycle = duty_cycle > this->max_pwm.get() ? this->max_pwm.get() : duty_cycle;

        // Set CCR register
        // The ccr register compares its value to the timer and outputs a signal based on the result
        // The arr register sets the limit for when the timer register resets to 0.
        if (this->ccr && this->arr) {
            *(this->ccr) = static_cast<uint32_t>(duty_cycle * (*(this->arr) + 1));
        }
    }

}
