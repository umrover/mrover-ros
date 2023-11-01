#pragma once

#include <config.hpp>
#include <hardware.hpp>
#include <units.hpp>

#include "main.h"

namespace mrover {

    struct BrushedMotorWriter {
    public:
        explicit BrushedMotorWriter(TIM_HandleTypeDef* timer);
        void target_target_duty_cycle(const Config& config, Percent output);
        void write();

    private:
        inline void set_direction_pins(Percent duty_cycle);
        inline void set_duty_cycle(Percent duty_cycle);

        Percent max_duty_cycle;
        Pin forward_pin{};
        Pin reverse_pin{};
        TIM_HandleTypeDef* timer{};
        volatile std::uint32_t* arr{};
        volatile std::uint32_t* ccr{};
        std::uint32_t channel{};
        Percent target_duty_cycle;
    };

} // namespace mrover
