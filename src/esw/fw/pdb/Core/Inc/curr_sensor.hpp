#pragma once

#include "stm32g4xx_hal.h"

#include "adc_sensor.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// Part Link
// https://www.digikey.com/en/products/detail/texas-instruments/TMCS1108A4BQDR/13692775
// Sensitivity: 400 mV/A
// Divide VCC by 5 to scale to our supply voltage

namespace mrover {

    class CurrentSensor {
    public:
        CurrentSensor() = default;

        CurrentSensor(ADCSensor* adc_sensor, uint8_t channel);

        void update_current();

        float get_current();

    private:
        ADCSensor* m_adc_sensor;
        uint8_t m_channel; // channel of adc sensor

        float m_current{};
    };

} // namespace mrover
