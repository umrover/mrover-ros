//OUR FUNCTIONS :)

#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"
#include "spectral.hpp"
#include "i2c_error_helper.hpp"

namespace mrover {

    Spectral::Spectral(std::shared_ptr<SMBus<uint8_t, uint16_t>> i2c_bus, std::shared_ptr<I2CMux> i2c_mux, uint8_t i2c_mux_channel)
    	: m_i2c_bus(i2c_bus),
    	  m_i2c_mux(i2c_mux),
		  m_i2c_mux_channel(i2c_mux_channel) {}

    void Spectral::reboot(){
    	m_i2c_bus->reboot();
    }

    void Spectral::poll_status_reg(){
    	m_i2c_mux->set_channel(m_i2c_mux_channel);
    	m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
    }

    void Spectral::init(){
    	assert(!m_initialized);
    	m_i2c_mux->set_channel(m_i2c_mux_channel);

    	// TODO (Alan) figure out if you need to actually poll the
    	// status reg and if while loops will cause potential hangs.
    	uint8_t control_data = 0x28;
    	// Control_data = 0x28
    	// RST is 0, so no reset is done
		// INT Is 0, so no interrupt
		// GAIN is 0b10, so it is 16x sensor channel gain
		// BANK is 0b10, so data conversion is Mode 2
		// DATA_RDY is 0 and RSVD is 0
//    	auto status = m_i2c_bus->blocking_transact<uint16_t, uint8_t>(SPECTRAL_7b_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
//		if(!status || status.value() & I2C_AS72XX_SLAVE_TX_VALID != 0){
//			// Try again
//			status = m_i2c_bus->blocking_transact<uint16_t, uint8_t>(SPECTRAL_7b_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
//			if(!status || status.value() & I2C_AS72XX_SLAVE_TX_VALID != 0){
//				// Failed. Unable to init...
//				m_error = true;
//				return;
//			}
//		}

    	// Waiting on semaphore replaces directly polling status register.
		osSemaphoreAcquire(spectral_write_status, osWaitForever);
		m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, CONTROL_SETUP_REG);

		osSemaphoreAcquire(spectral_write_status, osWaitForever);
		m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, control_data);

		osSemaphoreAcquire(spectral_write_status, osWaitForever);
		// Integration time = 2.8ms & 0xFF
		uint8_t int_time_multiplier = 0xFF;
		m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, int_time_multiplier);

		m_initialized = true;
		m_error = false;
    }

    void Spectral::update_channel_data() {
    	if (!m_initialized) {
    		init();
    	}
    	if (!m_initialized) {
    		return;
    	}
    	m_i2c_mux->set_channel(m_i2c_mux_channel);

//    	auto status = m_i2c_bus->blocking_transact<uint16_t, uint8_t>(SPECTRAL_7b_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
//    	if(!status || status.value() & I2C_AS72XX_SLAVE_TX_VALID != 0){
//    		// Failed. Unable to update data. We could just set the data to all 0 here.
//    		m_error = true;
//    		return;
//    	}

    	for(uint8_t i = 0; i < CHANNEL_DATA_LENGTH; ++i){
    		uint8_t msb_reg_addr = CHANNEL_V_HIGH + i * 2;
    		uint8_t lsb_reg_addr = CHANNEL_V_HIGH + i * 2 + 1;

    		// Technically we should we waiting/polling before trying to read from these addresses...
    		osSemaphoreAcquire(spectral_write_status, osWaitForever);
    		m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, msb_reg_addr);

    		osSemaphoreAcquire(spectral_read_status, osWaitForever);
    		auto msb_result = m_i2c_bus->blocking_receive(SPECTRAL_7b_ADDRESS);

    		osSemaphoreAcquire(spectral_write_status, osWaitForever);
    		m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, lsb_reg_addr);

    		osSemaphoreAcquire(spectral_read_status, osWaitForever);
			auto lsb_result = m_i2c_bus->blocking_receive(SPECTRAL_7b_ADDRESS);

    		if(msb_result && lsb_result){
    			channel_data[i] = (((uint16_t)msb_result << 8) | lsb_result);
    		}
    		else {
    			m_error = true;
    			//return;
    			throw mrover::I2CRuntimeError("update_channel_data failed: msb_result or lsb_result was not acquired.");
    		}
    	}
    	m_error = false;
    }


    uint16_t Spectral::get_channel_data(uint8_t channel) {
    	return channel_data.at(channel);
    }

    bool Spectral::is_error() {
    	return m_error;
    }
} // namespace mrover



