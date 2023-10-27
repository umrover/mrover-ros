#pragma once

#include <units.hpp>
#include <config.hpp>
#include <hardware.hpp>

#include "main.h"

namespace mrover {

    struct BrushedMotorWriter {
    public:
        explicit BrushedMotorWriter(TIM_HandleTypeDef* timer);
        void set_tgt(const Config& config, Dimensionless output);
        void write();

    private:
        inline void set_direction_pins(double duty_cycle);
        inline void set_pwm(double duty_cycle);

        Dimensionless max_pwm;
        Pin forward_pin{};
        Pin reverse_pin{};
        TIM_HandleTypeDef *timer{};
        volatile uint32_t *arr{};
        volatile uint32_t *ccr{};
        uint32_t channel{};
        double tgt_duty_cycle{};
    };

} // namespace mrover

