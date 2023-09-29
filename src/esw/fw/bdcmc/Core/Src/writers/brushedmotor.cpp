#include "writer.hpp"

#include <config.hpp>
#include <hardware.hpp>

#include "main.h"

extern TIM_HandleTypeDef htim3;

namespace mrover {

    void BrushedMotorWriter::init(const Config& config) {
        // Flag writer object as initialized
        this->initialized = true;

        // Define Writer Constants
        this->timer = &htim3;
        this->channel = TIM_CHANNEL_1;
        this->arr = &(TIM3->ARR);
        this->ccr = &(TIM3->CCR1);

        // Direction pins
        this->forward_pin = Pin(GPIOA, GPIO_PIN_0);
        this->reverse_pin = Pin(GPIOA, GPIO_PIN_1);

        HAL_TIM_PWM_Start(this->timer, this->channel);
    }

    void BrushedMotorWriter::write_output(const Config& config, Volts output) {
        // Force init on first call
        if (!this->initialized) this->init(config);

        // Set direction pins/duty cycle
        double duty_cycle = (output / config.getMaxVoltage()).get();
        this->set_direction_pins(duty_cycle);
        this->set_pwm(duty_cycle);
    }

    void BrushedMotorWriter::set_direction_pins(double duty_cycle) {
        this->forward_pin.write((duty_cycle > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        this->reverse_pin.write((duty_cycle < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    void BrushedMotorWriter::set_pwm(double duty_cycle) {
        // Obtain magnitude of duty_cycle and normalize
        duty_cycle *= duty_cycle > 0 ? 1 : -1;
        duty_cycle = duty_cycle > 1 ? 1 : duty_cycle;

        // Set CCR register
        // The ccr register compares its value to the timer and outputs a signal based on the result
        // The arr register sets the limit for when the timer register resets to 0. 
        *(this->ccr) = static_cast<uint32_t>(duty_cycle * (*(this->arr) + 1));
    }

}
