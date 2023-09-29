#include "writer.hpp"

#include "main.h"

extern TIM_HandleTypeDef htim3;

namespace mrover {

    void BrushedMotorWriter::init() {
        // Flag writer object as initialized
        this->initialized = true;

        // TODO Init PWM, Timers, etc.

    }

    void BrushedMotorWriter::write_output(const Config& config, Volts output) {
        if (!this->initialized) this->init();

        double duty_cycle = (output / config.getMaxVoltage()).get();
        this->set_direction_pins(duty_cycle);
        this->set_pwm(duty_cycle);
    }

    void BrushedMotorWriter::set_direction_pins(double duty_cycle) {
        this->forward_pin = duty_cycle > 0 ? 1 : 0;
        this->backward_pin = duty_cycle < 0 ? 1 : 0;
    }

    void BrushedMotorWrier::set_pwm(double duty_cycle) {
        // obtain magnitude of duty_cycle and normalize
        duty_cycle *= duty_cycle > 0 ? 1 : -1;
        duty_cycle = duty_cycle > 1 ? 1 : duty_cycle;

        // set CCR register
        // The ccr register compares its value to the timer and outputs a signal based on the result
        // The arr register sets the limit for when the timer register resets to 0. 
        *(this->ccr) = duty_cycle * (*(this->arr) + 1);
    }

}
