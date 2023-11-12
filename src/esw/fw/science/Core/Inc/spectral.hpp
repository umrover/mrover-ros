// datasheet for spectral sensor: https://www.mouser.com/datasheet/2/588/AS7262_DS000486_2-00-1082195.pdf

// AS7262 Spectral sensor

#pragma once

#include "stm32g4xx_hal.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware.hpp"
#include "i2c_mux.hpp"
#include <memory>

namespace mrover {

    class Spectral {
    public:
    	Spectral() = default;

    	Spectral(std::shared_ptr<I2CMux> i2c_mux, uint8_t i2c_mux_channel);

        void update_channel_data(uint8_t channel);

        uint16_t get_channel_data(uint8_t channel);

    private:
        std::shared_ptr<I2CMux> m_i2c_mux;
        uint8_t m_i2c_mux_channel;
        std::array<uint16_t, 6> channel_data {};
    };

} // namespace mrover

