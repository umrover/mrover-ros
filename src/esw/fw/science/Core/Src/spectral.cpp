//OUR FUNCTIONS :)

#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"
#include "spectral.hpp"

extern I2C_HandleTypeDef hi2c1;

namespace mrover {

    Spectral::Spectral(std::shared_ptr<SMBus<uint8_t, uint16_t>> i2c_bus, std::shared_ptr<I2CMux> i2c_mux, uint8_t i2c_mux_channel)
    	: m_i2c_bus(i2c_bus),
    	  m_i2c_mux(i2c_mux),
		  m_i2c_mux_channel(i2c_mux_channel) {}

    void Spectral::reboot(){
    	m_i2c_bus->reboot();
    }

    void Spectral::poll_status_reg(bool write){
    	m_i2c_mux->set_channel(m_i2c_mux_channel);
    	while(1){
			m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
			uint8_t tx_buf[1];
			tx_buf[0] = I2C_AS72XX_SLAVE_STATUS_REG;
			uint8_t buf[1];
			HAL_I2C_Master_Transmit(&hi2c1, SPECTRAL_7b_ADDRESS, tx_buf, 1, 100);
			HAL_I2C_Master_Receive(&hi2c1, SPECTRAL_7b_ADDRESS << 1 | 1, buf, 1, 100);
			uint8_t status = buf[0];
			// Check if we are allowed to proceed with reads + writes
//			auto status = m_i2c_bus->blocking_receive(SPECTRAL_7b_ADDRESS);
			uint8_t test = (status & I2C_AS72XX_SLAVE_TX_VALID);
			if(write && test == 0){
				break;
			}
			else if(!write && (status & I2C_AS72XX_SLAVE_RX_VALID != 0)){
				break;
			}


			osDelay(10); // Non blocking delay

			// Need to be able to fail here and not run until task runs out of time
			// ie. need a timeout
    	}
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

    	virtual_write(CONTROL_SETUP_REG, control_data);
    	osDelay(50);
    	virtual_write(CONTROL_SETUP_REG, control_data);
    	osDelay(50);

    	// Integration time = 2.8ms & 0xFF
    	uint8_t int_time_multiplier = 0xFF;
    	virtual_write(INT_TIME_REG, int_time_multiplier);


		m_initialized = true;
		m_error = false;
    }

    void Spectral::update_channel_data() {
    	if (!m_initialized) {
    		init();
    		// If it is still not initialized, just return.
			if (!m_initialized) {
				return;
			}
    	}
    	m_i2c_mux->set_channel(m_i2c_mux_channel);

    	for(uint8_t i = 0; i < CHANNEL_DATA_LENGTH; ++i){
    		uint8_t msb_reg_addr = CHANNEL_V_HIGH + i * 2;
    		uint8_t lsb_reg_addr = CHANNEL_V_HIGH + i * 2 + 1;

    		auto msb_result = virtual_read(msb_reg_addr);
    		auto lsb_result = virtual_read(lsb_reg_addr);

			channel_data[i] = (((uint16_t)msb_result << 8) | lsb_result);
    	}
    	m_error = false;
    }


    uint16_t Spectral::get_channel_data(uint8_t channel) {
    	return channel_data.at(channel);
    }

    void Spectral::virtual_write(uint8_t virtual_reg, uint8_t data){
    	poll_status_reg(true);
    	uint8_t buf[2];
    	buf[0] = I2C_AS72XX_WRITE_REG;
    	buf[1] = (virtual_reg | 0x80);
//    	m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, buf); How to send multiple bytes?
    	HAL_I2C_Master_Transmit(&hi2c1, SPECTRAL_7b_ADDRESS << 1, buf, sizeof(buf), 100);

    	poll_status_reg(true);
    	buf[0] = I2C_AS72XX_WRITE_REG;
		buf[1] = data;
		HAL_I2C_Master_Transmit(&hi2c1, SPECTRAL_7b_ADDRESS << 1, buf, sizeof(buf), 100);
    }

    uint8_t Spectral::virtual_read(uint8_t virtual_reg){
    	// Read status register may not work quite how it is described in the datasheet
    	poll_status_reg(false);
		uint8_t buf[2];
		buf[0] = I2C_AS72XX_WRITE_REG;
		buf[1] = virtual_reg;
		HAL_I2C_Master_Transmit(&hi2c1, SPECTRAL_7b_ADDRESS << 1, buf, sizeof(buf), 100);

		poll_status_reg(false);
		m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, I2C_AS72XX_READ_REG);
		auto result = m_i2c_bus->blocking_receive(SPECTRAL_7b_ADDRESS);

		return result;
    }

    bool Spectral::is_error() {
    	return m_error;
    }
} // namespace mrover



