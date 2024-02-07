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

	constexpr static float RESISTANCE_25 = 10000.0;
	constexpr static float THRM_A = 3.3540170E-03;
	constexpr static float THRM_B = 2.5617244E-04;
	constexpr static float THRM_C = 2.1400943E-06;
	constexpr static float THRM_D = -7.2405219E-08;

    DiagTempSensor::DiagTempSensor(std::shared_ptr<ADCSensor> adc_sensor, uint8_t channel)
    	: m_adc_sensor(adc_sensor), m_channel(channel) {}

    void DiagTempSensor::update_temp() {
    	float measured_voltage = m_adc_sensor->get_raw_channel_value(m_channel) * 3.3f / 4096.0f;
    	m_temp = (THRM_A4 * powf(measured_voltage,4)) + (THRM_A3 * powf(measured_voltage,3)) + (THRM_A2 * powf(measured_voltage,2)) + (THRM_A1 *  measured_voltage) + THRM_A0;
    }

    void DiagTempSensor::update_science_temp() {
    		float adc_cnt = m_adc_sensor->get_raw_channel_value(m_channel);
    		float measured_voltage = m_adc_sensor->get_raw_channel_value(m_channel) * 3.3f / 4096.0f;
        	float measured_resistance = ((m_adc_sensor->get_raw_channel_value(m_channel) * 3.3f / 4096.0f) * RESISTANCE_25)/(3.3f - (m_adc_sensor->get_raw_channel_value(m_channel) * 3.3f / 4096.0f));
        	m_temp = 1/(THRM_A + THRM_B*log(measured_resistance/RESISTANCE_25) + THRM_C*log((measured_resistance/RESISTANCE_25)*(measured_resistance/RESISTANCE_25)) + THRM_D*(((measured_resistance/RESISTANCE_25)*(measured_resistance/RESISTANCE_25)*(measured_resistance/RESISTANCE_25))));
        	m_temp -= 273.15;
    }

    float DiagTempSensor::get_temp() {
    	return m_temp;
    }

} // namespace mrover

