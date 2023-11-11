#include <numbers>

#include "adc_sensor.hpp"

namespace mrover {

	ADCSensor::ADCSensor(ADC_HandleTypeDef *hadc, uint8_t channels)
    	: m_hadc(hadc), m_channels(channels) {
		m_values.resize(channels);
	}

    uint16_t ADCSensor::get_raw_channel_value(uint8_t channel) {
    	return m_values.at(channel);
    }

    void ADCSensor::update() {
    	HAL_ADC_Start_DMA(m_hadc, m_values.data(), m_channels);
    }


} // namespace mrover

