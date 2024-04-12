#pragma once

#include "stm32g4xx_hal.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "diag_temp_sensor.hpp"

namespace mrover {

	constexpr static float DEFAULT_MAX_HEATER_TEMP = 50.0f;

    class Heater {
    public:
    	Heater() = default;

    	Heater(DiagTempSensor const& diag_temp_sensor, Pin const& heater_pin);

        float get_temp();

        bool get_state();

        void enable_if_possible(bool enable);

        void update_temp_and_auto_shutoff_if_applicable();

        void turn_off_if_watchdog_not_fed();

        void change_shutoff_temp(float shutoff_temp);

        void set_auto_shutoff(bool enable);

    private:
        void feed_watchdog();

        DiagTempSensor m_diag_temp_sensor;
        Pin m_heater_pin;
        bool m_state {};
        bool m_auto_shutoff_enabled {};
        uint32_t m_last_time_received_message {};
        float m_max_heater_temp {DEFAULT_MAX_HEATER_TEMP};
    };

} // namespace mrover
