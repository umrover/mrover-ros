#include "i2c_mux.hpp"
// Hardware PCA9546A
namespace mrover {

	I2CMux::I2CMux(std::shared_ptr<SMBus<uint8_t, uint16_t>> i2c_bus, Pin reset_pin)
    	: m_i2c_bus(i2c_bus), m_reset_pin(reset_pin) {
		reset_pin.write(GPIO_PIN_SET);
	}

    void I2CMux::set_channel(uint8_t channel) {
    	uint8_t go_to_channel = 1 << channel;
    	auto result = m_i2c_bus->blocking_transact(MUX_7b_ADDRESS, go_to_channel);
    	if(result){
    		current_channel = go_to_channel;
    	}
    	else{
    		// Error handling
    	}

    	// Clear read and write semaphore so that tasks need to re-acquire them
    	if(osSemaphoreGetCount(spectral_read_status) > 0){
    		osSemaphoreAcquire(spectral_read_status, osWaitForever);
    	}

    	if(osSemaphoreGetCount(spectral_write_status) > 0){
			osSemaphoreAcquire(spectral_write_status, osWaitForever);
		}
    }
} // namespace mrover



