#pragma once

#include <cstdint>

#include "hardware.hpp"
#include "units/units.hpp"

namespace mrover {

    struct HBridge {
        HBridge() = default;

        explicit HBridge(TIM_HandleTypeDef* timer, Pin forward_pin, Pin reverse_pin);

        void write(Percent output) const;

        void change_max_pwm(Percent max_pwm);

    private:
        void set_direction_pins(Percent duty_cycle) const;

        void set_duty_cycle(Percent duty_cycle, Percent max_duty_cycle) const;

        Pin m_forward_pins{};
        Pin m_reverse_pins{};
        TIM_HandleTypeDef* m_timer{};
        std::uint32_t m_channel = TIM_CHANNEL_1;
        std::uint32_t volatile* m_arr_register{};
        std::uint32_t volatile* m_ccr_register{};
        Percent m_max_pwm{};
    };

} // namespace mrover
