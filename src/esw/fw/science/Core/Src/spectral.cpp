//OUR FUNCTIONS :)

#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"

#include "spectral.hpp"

namespace mrover {

	constexpr static uint8_t I2C_AS72XX_SLAVE_STATUS_REG = 0x00;
	constexpr static uint8_t I2C_AS72XX_SLAVE_TX_VALID = 0x02;

    Spectral::Spectral(std::shared_ptr<I2CMux> i2c_mux, uint8_t i2c_mux_channel)
    	: m_i2c_mux(i2c_mux),
		  m_i2c_mux_channel(i2c_mux_channel) {}

    void Spectral::update_channel_data(uint8_t channel) {
    	// TODO -
    	m_i2c_mux->select_channel(m_i2c_mux_channel);
    	// TODO - implement select_channel using the following code:

    	// TODO - implement reading - here is some exmaple code from before
    	//REQUIRES: spectral is a Spectral device
    	//MODIFIES: spectral
    	//EFFECTS: spectral channel_data is loaded with data read in from the sensor
//    	void spectral_read(Spectral* spectral){
//
//    		//addresses for channel data
//    		char addresses[6] = {0x08, 0x09, 0x10, 0x11, 0x12, 0x13};
//    		const char DEV_ADDR = 0x49;
//
//    	    //check if it is ok to write (see page 20 of the spectral sensor datasheet)
//    	    volatile uint8_t status = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_STATUS_REG, DEV_ADDR);
//    	    if ((status & I2C_AS72XX_SLAVE_TX_VALID) != 0){
//    	        failed = true;
//    	        return;
//    		}
//
//    		for (int i = 0; i < 6; ++i){
//    			spectral->channel_data[i] = smbus_read_byte_data(spectral->smbus, addresses[i], DEV_ADDR);
//    		}
//
//    	}
    }

    uint16_t Spectral::get_channel_data(uint8_t channel) {
    	return channel_data.at(channel);
    }
} // namespace mrover



