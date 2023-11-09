#pragma once

#include "config.hpp"
#include "hardware.hpp"
#include "units/units.hpp"
#include "main.h"

namespace mrover {

    struct HBridgeWriter {
    public:
        HBridgeWriter() = default;

        explicit HBridgeWriter(TIM_HandleTypeDef* timer);

        void write(Config const& config, Percent output);

    private:
        void set_direction_pins(Percent duty_cycle);

        void set_duty_cycle(Percent duty_cycle, Percent max_duty_cycle);

        Pin m_forward_pins{};
        Pin m_reverse_pins{};
        TIM_HandleTypeDef* m_timer{};
        std::uint32_t m_channel{};
        volatile std::uint32_t* m_arr_register{};
        volatile std::uint32_t* m_ccr_register{};
    };

} // namespace mrover
