#pragma once

#include "stm32g4xx_hal.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware.hpp"

namespace mrover {

    class I2CMux {
    public:
    	I2CMux() = default;

    	I2CMux(I2C_HandleTypeDef* hi2c);

        void select_channel(uint8_t channel);

    private:
        SMBus m_i2c_bus;
    };

} // namespace mrover

