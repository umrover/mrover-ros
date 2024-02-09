#pragma once

#include <cstdint>

#include "hardware.hpp"
#include "units/units.hpp"

namespace mrover {

    struct HBridge {
        HBridge() = default;

        explicit HBridge(TIM_HandleTypeDef* timer, Pin dir_pin);

        void write(Percent output) const;

        void change_max_pwm(Percent max_pwm);

    private:
        void set_direction_pin(Percent duty_cycle) const;

        void set_duty_cycle(Percent duty_cycle, Percent max_duty_cycle) const;

        Pin m_dir_pin{};
        TIM_HandleTypeDef* m_timer{};
        std::uint32_t m_channel = TIM_CHANNEL_1;
        Percent m_max_pwm{};
    };

} // namespace mrover
