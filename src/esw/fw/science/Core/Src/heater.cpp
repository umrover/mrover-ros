#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"

#include "heater.hpp"




namespace mrover {

    constexpr static int MAX_HEATER_WATCHDOG_TICK = 1000;

    Heater::Heater(Thermistor const& thermistor, Pin const& heater_pin)
    	: m_thermistor(std::move(thermistor)),
		  m_heater_pin(std::move(heater_pin)),
		  m_state(false),
		  m_auto_shutoff_enabled(true),
		  m_last_time_received_message(0)
	   {}

    float Heater::get_temp() {
    	return m_thermistor.get_temp();
    }

    bool Heater::get_state() {
    	return m_state;
    }

    void Heater::enable_if_possible(bool enable) {
    	if (enable && m_auto_shutoff_enabled && get_temp() >= m_max_heater_temp) {
    		// The only time you don't service the request is if auto_shutoff is enabled
    		// and they want to turn it on when past the heater temp
			m_state = false;
    	}
    	else {
    		m_state = enable;
    	}

    	m_heater_pin.write(m_state ? GPIO_PIN_SET : GPIO_PIN_RESET);

    	feed_watchdog();
    }

    void Heater::update_temp_and_auto_shutoff_if_applicable() {
    	m_thermistor.update_science_temp();
    	if (m_state && m_auto_shutoff_enabled && (get_temp() >= m_max_heater_temp)) {
			m_state = false;
			m_heater_pin.write(GPIO_PIN_RESET);
		}
    }

    void Heater::turn_off_if_watchdog_not_fed() {
    	if (m_state) {
			bool watchdog_is_fed_recently = (HAL_GetTick() - m_last_time_received_message) <= MAX_HEATER_WATCHDOG_TICK;
			if (!watchdog_is_fed_recently) {
				m_state = false;
				m_heater_pin.write(GPIO_PIN_RESET);
			}
    	}
    }

    void Heater::change_shutoff_temp(float shutoff_temp) {
    	m_max_heater_temp = shutoff_temp;
    }

    void Heater::feed_watchdog() {
    	m_last_time_received_message = HAL_GetTick();
    }

    void Heater::set_auto_shutoff(bool enable) {
    	m_auto_shutoff_enabled = enable;
    }

} // namespace mrover