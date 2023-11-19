#include "i2c_mux.hpp"
// Hardware PCA9546A
namespace mrover {

	I2CMux::I2CMux(std::shared_ptr<SMBus> i2c_bus)
    	: m_i2c_bus(i2c_bus) {}

    void I2CMux::set_channel(uint8_t channel) {
    	uint8_t go_to_channel = 1 << channel;
    	auto result = m_i2c_bus->blocking_transact<uint16_t, uint8_t>(MUX_7b_ADDRESS, go_to_channel);
    	if(result){
    		current_channel = go_to_channel;
    	}
    	else{
    		// Error handling
    	}
    }
} // namespace mrover



