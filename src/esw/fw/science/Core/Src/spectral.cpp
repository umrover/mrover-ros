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
    	for(int i = 0; i < 10; ++i){
//			m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
//			uint8_t tx_buf[1];
//			tx_buf[0] = I2C_AS72XX_SLAVE_STATUS_REG;
//			uint8_t buf[1];
//			HAL_I2C_Master_Transmit(&hi2c1, SPECTRAL_7b_ADDRESS, tx_buf, 1, 100);
//			HAL_I2C_Master_Receive(&hi2c1, SPECTRAL_7b_ADDRESS << 1 | 1, buf, 1, 100);
//			uint8_t status = buf[0];
			// Check if we are allowed to proceed with reads + writes
//			auto status = m_i2c_bus->blocking_receive(SPECTRAL_7b_ADDRESS);

			auto status = m_i2c_bus->blocking_transact(SPECTRAL_7b_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);

			if(status){
				switch(rw){
				case WRITE:
					if((status.value() & I2C_AS72XX_SLAVE_TX_VALID) == 0) return;
				case READ:
//					if((status.value() & I2C_AS72XX_SLAVE_RX_VALID) != 0) return;
					return;
				}
//				if(write && (status.value() & I2C_AS72XX_SLAVE_TX_VALID) == 0){
//					break;
//				}
//				else if(!write && (status.value() & I2C_AS72XX_SLAVE_RX_VALID != 0)){
//					break;
//				}
			}

			osDelay(5); // Non blocking delay
    	}

    	//throw mrover::I2CRuntimeError("SPECTRAL");
    	m_error = true;
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
		virtual_write(CONTROL_SETUP_REG, control_data);
		osDelay(50);

		// Integration time = 2.8ms & 0xFF
		uint8_t int_time_multiplier = 0xFF;
		virtual_write(INT_TIME_REG, int_time_multiplier);


		m_initialized = true;
		m_error = false;
    }

    bool Spectral::update_channel_data() {
    	if (!m_initialized) {
    		init();
    		// If it is still not initialized, just return.
			if (!m_initialized) {
				return false;
			}
    	}
    	m_i2c_mux->set_channel(m_i2c_mux_channel);

    	for(uint8_t i = 0; i < CHANNEL_DATA_LENGTH; ++i){
    		// big endian, so msb is at lowest addr (which we read first)
    		/* UNCOMMENT ONCE CAN MESSAGE IS FIXED - UNABLE TO SEND 6 FLOAT CURRENTLY
			uint8_t msb_reg_addr_0 = CHANNEL_V_HIGH + i * 4;
			uint8_t msb_reg_addr_1 = CHANNEL_V_HIGH + i * 4 + 1;
			uint8_t msb_reg_addr_2 = CHANNEL_V_HIGH + i * 4 + 2;
			uint8_t msb_reg_addr_3 = CHANNEL_V_HIGH + i * 4 + 3;

			uint32_t msb_result_0 = virtual_read(msb_reg_addr_0);
			uint32_t msb_result_1 = virtual_read(msb_reg_addr_1);
			uint32_t msb_result_2 = virtual_read(msb_reg_addr_2);
			uint32_t msb_result_3 = virtual_read(msb_reg_addr_3);
    		if(m_error) return false;

    		uint32_t combined_val = (msb_result_0 << 24) | (msb_result_1 << 16) | (msb_result_2 << 8) | (msb_result_3);
			channel_data[i] = std::bit_cast<float>(combined_val);
			*/
//    		if (!msb_result || !lsb_result){
//    			//throw mrover::I2CRuntimeError("MSB or LSB result not read in update_channel_data");
//    			m_error = true;
//    			return false;
//    		}

    		uint8_t msb_reg_addr = CHANNEL_V_HIGH + i * 2;
			uint8_t lsb_reg_addr = CHANNEL_V_HIGH + i * 2 + 1;

			uint8_t msb_result = virtual_read(msb_reg_addr);
			uint8_t lsb_result = virtual_read(lsb_reg_addr);
			if(m_error) return false;


			uint16_t msb_result_int = msb_result;
			uint16_t lsb_result_int = lsb_result;
			channel_data[i] = ((msb_result_int << 8) | lsb_result_int);

    	}
    	m_error = false;
    	return true;
    }


    uint16_t Spectral::get_channel_data(uint8_t channel) {
    	return channel_data.at(channel);
    }

    void Spectral::virtual_write(uint8_t virtual_reg, uint8_t data){
		poll_status_reg(I2C_OP::WRITE);
		uint8_t buf[2];
		buf[0] = I2C_AS72XX_WRITE_REG;
		buf[1] = (virtual_reg | 0x80);
		m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, buf[0]);
		// How to send multiple bytes?
    	HAL_I2C_Master_Transmit(&hi2c1, SPECTRAL_7b_ADDRESS << 1, buf, sizeof(buf), 100);

		poll_status_reg(I2C_OP::WRITE);
		buf[0] = I2C_AS72XX_WRITE_REG;
		buf[1] = data;
//		m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, buf[0]);

		HAL_I2C_Master_Transmit(&hi2c1, SPECTRAL_7b_ADDRESS << 1, buf, sizeof(buf), 100);

    }

    uint8_t Spectral::virtual_read(uint8_t virtual_reg){// -> std::optional<uint16_t>{
    	// Read status register may not work quite how it is described in the datasheet
    	poll_status_reg(I2C_OP::READ);
		uint8_t buf[2];
		buf[0] = I2C_AS72XX_WRITE_REG;
		buf[1] = virtual_reg;
		HAL_I2C_Master_Transmit(&hi2c1, SPECTRAL_7b_ADDRESS << 1, buf, sizeof(buf), 100);
		// UNABLE TO USE this format because TSend is 1 byte, need to double up
		// TODO resolve this somehow...
//		m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, buf[0]);


		poll_status_reg(I2C_OP::READ);

		auto result = m_i2c_bus->blocking_transact(SPECTRAL_7b_ADDRESS, I2C_AS72XX_READ_REG);
//		m_i2c_bus->blocking_transmit(SPECTRAL_7b_ADDRESS, I2C_AS72XX_READ_REG);
//		auto result = m_i2c_bus->blocking_receive(SPECTRAL_7b_ADDRESS);
		if(!result){
			m_error = true;
			return 0;
		}
		return result.value();
    }

    bool Spectral::is_error() {
    	return m_error;
    }
} // namespace mrover



