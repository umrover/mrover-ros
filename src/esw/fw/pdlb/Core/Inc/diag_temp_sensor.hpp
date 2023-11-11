#pragma once

#include "stm32g4xx_hal.h"

#include "adc_sensor.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace mrover {

    class DiagTempSensor {
    public:
    	DiagTempSensor() = default;

    	DiagTempSensor(ADCSensor* adc_sensor, uint8_t channel);

        void update_temp();

        float get_temp();

    private:
        ADCSensor* m_adc_sensor;
        uint8_t m_channel; // channel of adc sensor

        float m_temp{};
    };

} // namespace mrover

