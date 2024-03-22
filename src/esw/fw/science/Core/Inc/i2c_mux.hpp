// PCA9546A I2C Multiplexer/Switch
// Datasheet: https://www.ti.com/lit/ds/symlink/pca9546a.pdf

#pragma once

#include "stm32g4xx_hal.h"

#include "cmsis_os2.h"

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "hardware_i2c.hpp"

#include <numbers>
#include "hardware.hpp"
#include "units/units.hpp"
#include <memory>

extern osSemaphoreId_t spectral_read_status;
extern osSemaphoreId_t spectral_write_status;

namespace mrover {

    class I2CMux {
    public:
    	I2CMux() = default;

    	I2CMux(std::shared_ptr<SMBus<uint8_t, uint8_t>> i2c_bus, Pin reset_pin);

        void set_channel(uint8_t channel);

    private:
        std::shared_ptr<SMBus<uint8_t, uint8_t>> m_i2c_bus;
        uint8_t current_channel = 0;
        constexpr static std::uint16_t MUX_7b_ADDRESS = 0x70;
        Pin m_reset_pin;
    };

} // namespace mrover

