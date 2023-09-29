#pragma once

#include <units.hpp>
#include <config.hpp>

#include "main.h"

namespace mrover {

    struct BrushedMotorWriter {
    public:
        void write_output(const Config& config, Volts output);
    private:
        void init(const Config& config);
        TIM_HandleTypeDef *timer;
        volatile uint32_t *arr;
        volatile uint32_t *ccr;
        uint32_t channel;
        bool initialized{false};
    };

} // namespace mrover

