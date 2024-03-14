#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"

#include <diag_temp_sensor.hpp>

// TMP6431DECR

namespace mrover {

    constexpr static float DIAG_TEMP_COEFFICIENT = 0.0064f;
    constexpr static float DIAG_TEMP_25_DEGREE_RESISTANCE = 47000.0;
    constexpr static float THRM_A0 = -2.808429E+02;
    constexpr static float THRM_A1 = 1.186908E-02;
    constexpr static float THRM_A2 = -1.731036E-07;
	constexpr static float THRM_A3 = 1.517359E-12;
	constexpr static float THRM_A4 = -5.398440E-18;

    DiagTempSensor::DiagTempSensor(std::shared_ptr<ADCSensor> adc_sensor, uint8_t channel)
    	: m_adc_sensor(adc_sensor), m_channel(channel) {

    }

    void DiagTempSensor::update_temp() {
    	uint16_t adc_val = m_adc_sensor->get_raw_channel_value(m_channel);
    	float measured_volts = (adc_val * 3.3f) / 4096.0f;
    	float calculated_current = (3.3f - measured_volts) / (10000.0f);
    	float thermistor_resistance = measured_volts / calculated_current;

    	m_temp = (THRM_A4 * powf(thermistor_resistance,4)) + (THRM_A3 * powf(thermistor_resistance,3)) + (THRM_A2 * powf(thermistor_resistance,2)) + (THRM_A1 *  thermistor_resistance) + THRM_A0;
    }

    float DiagTempSensor::get_temp() {
    	return m_temp;
    }

} // namespace mrover

