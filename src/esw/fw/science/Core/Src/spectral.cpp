//OUR FUNCTIONS :)

#include <numbers>

#include "hardware.hpp"
#include "units/units.hpp"
#include "spectral.hpp"
#include <cassert>

extern I2C_HandleTypeDef hi2c1;

namespace mrover {

    Spectral::Spectral(std::shared_ptr<SMBus<uint8_t, uint8_t>> i2c_bus, std::shared_ptr<I2CMux> i2c_mux, uint8_t i2c_mux_channel)
    	: m_i2c_bus(i2c_bus),
    	  m_i2c_mux(i2c_mux),
		  m_i2c_mux_channel(i2c_mux_channel) {}

    void Spectral::reboot(){
    	m_i2c_bus->reboot();
    }

    void Spectral::poll_status_reg(I2C_OP rw){
    	m_i2c_mux->set_channel(m_i2c_mux_channel);
    	for(int i = 0; i < 100; ++i){
			auto status = m_i2c_bus->blocking_transact(SPECTRAL_7b_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);

			if (status.has_value()) {
				if (rw == READ) {
					if((status.value() & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
						m_error = false;
						return;
					}
				}
				else if (rw == WRITE) {
					if ((status.value() & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
						m_error = false;
						return;
					}
				}
			}
			else {
				m_error = true;
				m_initialized = false;
			}

			osDelay(5); // Non blocking delay
    	}

    	m_error = true;
    	m_initialized = false;
    }

    void Spectral::init(){
    	assert(!m_initialized);
    	m_i2c_mux->set_channel(m_i2c_mux_channel);

    	uint8_t control_data = 0x28;
    	// Control_data = 0x28
    	// RST is 0, so no reset is done
		// INT Is 0, so no interrupt
		// GAIN is 0b10, so it is 16x sensor channel gain
		// BANK is 0b10, so data conversion is Mode 2
		// DATA_RDY is 0 and RSVD is 0
		virtual_write(CONTROL_SETUP_REG, control_data);
		osDelay(50);

		// Integration time = 2.8ms & 0xFF
		uint8_t int_time_multiplier = 0xFF; //0xFF;
		virtual_write(INT_TIME_REG, int_time_multiplier);

		if (m_error) {
			return;
		}

		m_initialized = true;
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
    		// big endian, so msb is at lowest addr (which we read first)
    		uint32_t combined_val = 0;
    		for (int j = 0; j < 4; ++j) {
    			uint8_t msb_reg_addr = CHANNEL_V_CAL + i * 4 + j;
    			uint8_t msb_result = virtual_read(msb_reg_addr);
				if(m_error) {
					m_initialized = false;
					return;
				}
    			combined_val |= msb_result << ((3 - j) * 8);
    		}
    		float converted_val = 0;
    		memcpy(&converted_val, &combined_val, 4);
			channel_data[i] = converted_val;
    	}
    	m_error = false;
    	return;
    }


    float Spectral::get_channel_data(uint8_t channel) {
    	return channel_data.at(channel);
    }

    void Spectral::virtual_write(uint8_t virtual_reg, uint8_t data){
		poll_status_reg(I2C_OP::WRITE);
		uint8_t buf[2];
		buf[0] = I2C_AS72XX_WRITE_REG;
		buf[1] = (virtual_reg | 0x80);
    	HAL_I2C_Master_Transmit(&hi2c1, SPECTRAL_7b_ADDRESS << 1, buf, sizeof(buf), 100);

		poll_status_reg(I2C_OP::WRITE);
		buf[0] = I2C_AS72XX_WRITE_REG;
		buf[1] = data;

		HAL_I2C_Master_Transmit(&hi2c1, SPECTRAL_7b_ADDRESS << 1, buf, sizeof(buf), 100);

    }

    uint8_t Spectral::virtual_read(uint8_t virtual_reg){// -> std::optional<uint16_t>{
    	// Read status register may not work quite how it is described in the datasheet

    	auto status = m_i2c_bus->blocking_transact(SPECTRAL_7b_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);

    	if (status.has_value()) {
			if ((status.value() & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
				auto not_used = m_i2c_bus->blocking_transact(SPECTRAL_7b_ADDRESS, I2C_AS72XX_READ_REG);
			}
    	}
    	else {
    		m_error = true;
    		m_initialized = false;
    		return 0;
    	}

    	poll_status_reg(I2C_OP::WRITE);

    	if (m_error) {
    		return 0;
    	}
		uint8_t buf[2];
		buf[0] = I2C_AS72XX_WRITE_REG;
		buf[1] = virtual_reg;
		HAL_I2C_Master_Transmit(&hi2c1, SPECTRAL_7b_ADDRESS << 1, buf, sizeof(buf), 100);

		poll_status_reg(I2C_OP::READ);

		if (m_error) {
			return 0;
		}

		auto result = m_i2c_bus->blocking_transact(SPECTRAL_7b_ADDRESS, I2C_AS72XX_READ_REG);
		if(!result.has_value()){
			m_error = true;
			return 0;
		}
		m_error = false;
		return result.value();
    }

    bool Spectral::is_error() {
    	return m_error;
    }
} // namespace mrover