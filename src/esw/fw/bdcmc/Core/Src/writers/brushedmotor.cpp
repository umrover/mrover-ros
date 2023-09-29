#include "writer.hpp"

#include "main.h"
#include "config.hpp"

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

        HAL_TIM_PWM_Start(this->timer, this->channel);
    }

    void BrushedMotorWriter::write_output(const Config& config, Volts output) {
        if (!this->initialized) this->init(config);

        // TODO write PWM, etc.
        (void) output;
    }

}
