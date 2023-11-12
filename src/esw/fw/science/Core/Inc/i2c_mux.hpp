#pragma once

#include "stm32g4xx_hal.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware_i2c.hpp"

#include <numbers>
#include "hardware.hpp"
#include "units/units.hpp"
#include <memory>

namespace mrover {

    class I2CMux {
    public:
    	I2CMux() = default;

    	I2CMux(std::shared_ptr<SMBus> i2c_bus);

        void select_channel(uint8_t channel);

    private:
        std::shared_ptr<SMBus> m_i2c_bus;
    };

} // namespace mrover

