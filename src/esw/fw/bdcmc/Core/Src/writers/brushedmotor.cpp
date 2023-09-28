#include "writer.hpp"

#include "main.h"

extern TIM_HandleTypeDef htim3;

namespace mrover {

    void BrushedMotorWriter::init() {
        // Flag writer object as initialized
        this->initialized = true;

        // TODO Init PWM, Timers, etc.

    }

    void BrushedMotorWriter::write_output(Volts output) {
        if (!this->initialized) this->init();

        // TODO write PWM, etc.
        (void) output;
    }

}
