#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"

#include <diag_temp_sensor.hpp>

namespace mrover {

    constexpr static float DIAG_TEMP_COEFFICIENT = 0.0064f;
    constexpr static float DIAG_TEMP_25_DEGREE_RESISTANCE = 47000.0;
    constexpr static float THRM_A0 = -5.160732E+02;
    constexpr static float THRM_A1 = 6.831122E+02;
    constexpr static float THRM_A2 = -3.774928E+02;
	constexpr static float THRM_A3 = 1.159826E+02;
	constexpr static float THRM_A4 = -1.060407E+01;

    DiagTempSensor::DiagTempSensor(std::shared_ptr<ADCSensor> adc_sensor, uint8_t channel)
    	: m_adc_sensor(adc_sensor), m_channel(channel) {

    }

    void DiagTempSensor::update_temp() {
    	uint16_t adc_val = m_adc_sensor->get_raw_channel_value(m_channel);
    	float measured_volts = (adc_val * 3.3f) / 4096.0f;
    	m_temp = (THRM_A4 * powf(measured_volts,4)) + (THRM_A3 * powf(measured_volts,3)) + (THRM_A2 * powf(measured_volts,2)) + (THRM_A1 *  measured_volts) + THRM_A0;
    }

    float DiagTempSensor::get_temp() {
    	return m_temp;
    }

} // namespace mrover

