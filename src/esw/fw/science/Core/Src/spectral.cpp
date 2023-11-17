//OUR FUNCTIONS :)

#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"

#include "spectral.hpp"

namespace mrover {

	constexpr static uint8_t I2C_AS72XX_SLAVE_STATUS_REG = 0x00;
	constexpr static uint8_t I2C_AS72XX_SLAVE_TX_VALID = 0x02;

    Spectral::Spectral(std::shared_ptr<SMBus> i2c_bus, std::shared_ptr<I2CMux> i2c_mux, uint8_t i2c_mux_channel)
    	: m_i2c_bus(i2c_bus),
    	  m_i2c_mux(i2c_mux),
		  m_i2c_mux_channel(i2c_mux_channel) {}

    void Spectral::init(){
    	// TODO (Alan) figure out if you need to actually poll the
    	// status reg and if while loops will cause potential hangs.
    	uint8_t control_data = 0x28;
    	// Control_data = 0x28
    	// RST is 0, so no reset is done
		// INT Is 0, so no interrupt
		// GAIN is 0b10, so it is 16x sensor channel gain
		// BANK is 0b10, so data conversion is Mode 2
		// DATA_RDY is 0 and RSVD is 0
    	auto status = m_i2c_bus->blocking_transact<uint16_t, uint8_t>(SPECTRAL_7b_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if(!status || status.value() & I2C_AS72XX_SLAVE_TX_VALID != 0){
			// Try again
			status = m_i2c_bus->blocking_transact<uint16_t, uint8_t>(SPECTRAL_7b_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
			if(!status || status.value() & I2C_AS72XX_SLAVE_TX_VALID != 0){
				// Failed. Unable to init...
				return;
			}
		}

		auto result = m_i2c_bus->blocking_transact<uint16_t, uint8_t>(SPECTRAL_7b_ADDRESS, CONTROL_SETUP_REG);
		result = m_i2c_bus->blocking_transact<uint16_t, uint8_t>(SPECTRAL_7b_ADDRESS, control_data);
		// Integration time = 2.8ms & 0xFF
		uint8_t int_time_multiplier = 0xFF;
		result = m_i2c_bus->blocking_transact<uint16_t, uint8_t>(SPECTRAL_7b_ADDRESS, int_time_multiplier);
    }

    void Spectral::update_channel_data(uint8_t channel) {
    	// TODO -
    	m_i2c_mux->set_channel(m_i2c_mux_channel);
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

    	auto status = m_i2c_bus->blocking_transact<uint16_t, uint8_t>(SPECTRAL_7b_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
    	if(!status || status.value() & I2C_AS72XX_SLAVE_TX_VALID != 0){
    		// Failed. Unable to update data. We could just set the data to all 0 here.
    		return;
    	}

    	for(uint8_t i = 0; i < CHANNEL_DATA_LENGTH; ++i){
    		uint8_t msb_reg_addr = CHANNEL_V_HIGH + i * 2;
    		uint8_t lsb_reg_addr = CHANNEL_V_HIGH + i * 2 + 1;

    		auto msb_result = m_i2c_bus->blocking_transact<uint16_t, uint8_t>(SPECTRAL_7b_ADDRESS, msb_reg_addr);
    		auto lsb_result = m_i2c_bus->blocking_transact<uint16_t, uint8_t>(SPECTRAL_7b_ADDRESS, lsb_reg_addr);
    		if(msb_result && lsb_result){
    			channel_data[i] = (((uint16_t)msb_result.value() << 8) | lsb_result.value());
    		}
    	}


    }

    uint16_t Spectral::get_channel_data(uint8_t channel) {
    	return channel_data.at(channel);
    }
} // namespace mrover



