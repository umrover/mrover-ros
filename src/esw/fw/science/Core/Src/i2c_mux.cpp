#include "i2c_mux.hpp"

namespace mrover {

	I2CMux::I2CMux(std::shared_ptr<SMBus> i2c_bus)
    	: m_i2c_bus(i2c_bus) {}

    void I2CMux::select_channel(uint8_t channel) {
    	 uint8_t formatted = 1 << channel;
//    	 m_i2c_bus.transact(channel, 0x70); // TODO - verify if this is correct
    }
} // namespace mrover



