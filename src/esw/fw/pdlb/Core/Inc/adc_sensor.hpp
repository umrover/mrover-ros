#pragma once

#include "stm32g4xx_hal.h"

#include "hardware.hpp"

namespace mrover {

    class ADCSensor {
    public:
    	ADCSensor() = default;

    	ADCSensor(ADC_HandleTypeDef *hadc, uint8_t channels);

        uint16_t get_raw_channel_value(uint8_t channel);

        // TODO - write Voltage too

        void update();

    private:
        ADC_HandleTypeDef* m_hadc;
        uint8_t m_channels;
        std::vector<uint16_t> m_values;
    };

} // namespace mrover

