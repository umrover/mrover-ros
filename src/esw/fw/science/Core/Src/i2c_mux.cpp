//OUR FUNCTIONS :)

#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"

#include "i2c_mux.hpp"

namespace mrover {

	I2CMux::I2CMux(I2C_HandleTypeDef* hi2c)
    	: m_i2c_bus(SMBus(hi2c)) {}

    void I2CMux::select_channel(uint8_t channel) {
    	 uint8_t formatted = 1 << channel;
//    	 m_i2c_bus.transact(channel, 0x70); // TODO - verify if this is correct
    }
} // namespace mrover



