#pragma once

#include <cstdint>

#include "hardware.hpp"
#include "units/units.hpp"

namespace mrover {

    class HBridge {
        Pin m_positive_pin{};
        Pin m_negative_pin{};
        TIM_HandleTypeDef* m_timer{};
        std::uint32_t m_channel = TIM_CHANNEL_1;
        Percent m_max_pwm{};
        bool m_is_inverted = false;

    public:
        HBridge() = default;

        explicit HBridge(TIM_HandleTypeDef* timer, Pin positive_pin, Pin negative_pin);

        auto write(Percent output) const -> void;

        auto set_direction_pins(Percent duty_cycle) const -> void;

        auto set_duty_cycle(Percent duty_cycle, Percent max_duty_cycle) const -> void;

        auto change_max_pwm(Percent max_pwm) -> void;

        auto change_inverted(bool inverted) -> void;
    };

} // namespace mrover
