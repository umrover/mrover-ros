#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"

#include "curr_sensor.hpp"

namespace mrover {

    constexpr static float DIAG_CURR_VCC = 3.3f;
    constexpr static float DIAG_CURR_V_PER_AMP = 0.4f;
    constexpr static float DIAG_CURR_V_S = 3.3f;
    constexpr static float DIAG_CURR_S = 0.2f;

    CurrentSensor::CurrentSensor(std::shared_ptr<ADCSensor> adc_sensor, uint8_t channel)
    	: m_adc_sensor(adc_sensor), m_channel(channel) {}

    void CurrentSensor::update_current() {
    	uint16_t adc_val = m_adc_sensor->get_raw_channel_value(m_channel);
		float measured_volts = (adc_val * 3.3f) / 4096.0f;

		m_current = (DIAG_CURR_V_S / 2.0f - measured_volts) / -DIAG_CURR_S;
    }

    float CurrentSensor::get_current() {
    	return m_current;
    }


} // namespace mrover
