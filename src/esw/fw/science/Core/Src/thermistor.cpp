#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"

#include <thermistor.hpp>

namespace mrover {

	constexpr static float RESISTANCE_25 = 10000.0;
	constexpr static float BIAS_RESISTANCE = 10000.0; // circuit is ground - thermistor - analog input - bias resistor - 3.3v
	constexpr static float THRM_A = 3.3540170E-03;
	constexpr static float THRM_B = 2.5617244E-04;
	constexpr static float THRM_C = 2.1400943E-06;
	constexpr static float THRM_D = -7.2405219E-08;

    Thermistor::Thermistor(std::shared_ptr<ADCSensor> adc_sensor, uint8_t channel)
    	: m_adc_sensor(adc_sensor), m_channel(channel) {}

    void Thermistor::update_science_temp() {
    		// Magic number used to calibrate thermistor temperature
    		float adc_cnt = m_adc_sensor->get_raw_channel_value(m_channel);
    		float measured_voltage = m_adc_sensor->get_raw_channel_value(m_channel) * 3.3f / 4096.0f;
        	float measured_resistance = (measured_voltage * BIAS_RESISTANCE)/(3.3f - measured_voltage);
        	m_temp = 1/(THRM_A + THRM_B*log(measured_resistance/RESISTANCE_25) + THRM_C*log((measured_resistance/RESISTANCE_25)*(measured_resistance/RESISTANCE_25)) + THRM_D*(((measured_resistance/RESISTANCE_25)*(measured_resistance/RESISTANCE_25)*(measured_resistance/RESISTANCE_25))));
       		m_temp -= 273.15;
    }

    float Thermistor::get_temp() {
    	return m_temp;
    }

} // namespace mrover