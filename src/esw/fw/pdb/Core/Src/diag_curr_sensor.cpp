#pragma once

#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"

#include "curr_sensor.hpp"

namespace mrover {

    // NOTE: Change This For Each Motor Controller
    constexpr static std::uint8_t DEVICE_ID = 0x1;

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x0;

    constexpr static float DIAG_CURR_VCC = 3.3f;
    constexpr static float DIAG_CURR_V_PER_AMP = 0.4f;
    constexpr static float DIAG_CURR_V_S = 3.3f;
    constexpr static float DIAG_CURR_S = 0.2f;

    CurrentSensor::CurrentSensor(ADCSensor* adc_sensor, uint8_t channel)
    	: m_adc_sensor(adc_sensor), m_channel(channel) {}

    // TODO - rename everything to current sensor
    void CurrentSensor::update_current() {
    	uint16_t adc_val = get_adc_sensor_value(m_adc_sensor, m_channel);
		float measured_volts = (adc_val * 3.3f) / 4096.0f;

		m_current = (DIAG_CURR_V_S / 2.0f - measured_volts) / -DIAG_CURR_S;
    }

    float CurrentSensor::get_current() {
    	return m_current;
    }


} // namespace mrover
