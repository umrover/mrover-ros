#pragma once

#include <units.hpp>
#include <config.hpp>
#include <hardware.hpp>

#include "main.h"

namespace mrover {

    struct BrushedMotorWriter {
    public:
        void write_output(const Config& config, Volts output);
    private:
        void init(const Config& config);
        Pin forward_pin{};
        Pin reverse_pin{};
        TIM_HandleTypeDef *timer{};
        volatile uint32_t *arr{};
        volatile uint32_t *ccr{};
        uint32_t channel{};
        bool initialized{false};
        inline void set_direction_pins(double duty_cycle);
        inline void set_pwm(double duty_cycle);
    };

} // namespace mrover

