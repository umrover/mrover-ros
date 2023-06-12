#include "spectral.h"

// REQUIRES: i2c is the i2c channel
// and uart is the debugging UART channel or NULL,
// and dma tells if DMA is enabled
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Spectral object
Spectral *new_spectral(SMBus *smbus)
{
    Spectral *spectral = malloc(sizeof(Spectral));
    spectral->smbus = smbus;
	for (uint8_t i = 0; i < SPECTRAL_CHANNELS; ++i) {
		spectral->channel_data[i] = 0;
	}
    return spectral;
}

// REQUIRES: spectral is a Spectral object
// MODIFIES: nothing
// EFFECTS: Initializes the spectral device
void initialize_spectral(Spectral *spectral)
{
	// 0x28 means the following:
	// RST is 0, so no reset is done
	// INT Is 0, so no interrupt
	// GAIN is 0b10, so it is 16x sensor channel gain
	// BANK is 0b10, so data conversion is Mode 2
	// DATA_RDY is 0 and RSVD is 0
	virtual_write_spectral(spectral, CONTROL_SET_UP, 0x28);  // runs twice to account for status miss
	HAL_Delay(50);
	virtual_write_spectral(spectral, CONTROL_SET_UP, 0x28);  // converts data bank to 2
	// Integration time is 0xFF * 2.8ms
	HAL_Delay(50);
	virtual_write_spectral(spectral, INT_TIME, 0xFF);  // increases integration time
}

// REQUIRES: spectral is an object
// MODIFIES: spectral.channels array
// EFFECTS: Updates values of spectral struct's channels array with data from spectral sensor
void update_spectral_all_channel_data(Spectral *spectral) {
	uint8_t *error_flag = 0;
	// Update ALL channels in the spectral struct
	for(int i = 0; i < SPECTRAL_CHANNELS; ++i) {
		update_spectral_channel_data(spectral, i, error_flag);
	}
}

// REQUIRES: spectral is an object and 0 <= channel < 6
// MODIFIES: spectral.channels array
// EFFECTS: Updates values of spectral struct's channels array with data from spectral sensor
void update_spectral_channel_data(Spectral *spectral, uint8_t channel, uint8_t *error_flag) {


	if (0 <= channel && channel < 6) {
		uint8_t START_REG = RAW_VALUE_RGA_HIGH;

		uint8_t msb = START_REG + channel * 2;
		uint8_t lsb = START_REG + channel * 2 + 1;

		 uint16_t high = (virtual_read_spectral(spectral, msb, error_flag) & 0xFF) << 8;
		 spectral->channel_data[channel] = high | (virtual_read_spectral(spectral, lsb, error_flag) & 0xFF);
	}

}

// REQUIRES: spectral is a Spectral object and 0 <= channel < 6
// MODIFIES: nothing
// EFFECTS: Returns the spectral data of a particular channel
uint16_t get_spectral_channel_data(Spectral *spectral, uint8_t channel)
{
	return spectral->channel_data[channel];

}

/* PRIVATE FUNCTIONS */

// REQUIRES: spectral is a Spectral object,
// v_reg is the virtual register,
// and data is the data to pass
// MODIFIES: nothing
// EFFECTS: Writes to the virtual register as explained in page 18-20 of the datasheet
void virtual_write_spectral(Spectral *spectral, uint8_t v_reg, uint8_t data) {
    uint8_t status;

	while(1) {
		status = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_STATUS_REG, DEVICE_SLAVE_ADDRESS);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}
	smbus_write_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_WRITE_REG, (v_reg | 0x80), DEVICE_SLAVE_ADDRESS);

	while(1) {
		status = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_STATUS_REG, DEVICE_SLAVE_ADDRESS);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}
	smbus_write_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_WRITE_REG, data, DEVICE_SLAVE_ADDRESS);
}

// REQUIRES: spectral is a Spectral object and
// v_reg is the virtual register
// MODIFIES: nothing
// EFFECTS: Returns the value read from the virtual register
// as explained in page 18-20 of the datasheet
uint8_t virtual_read_spectral(Spectral *spectral, uint8_t v_reg, uint8_t *error) {
    // Taken from datasheet
    uint8_t has_error;
	uint8_t d;
	uint8_t counter;
	has_error = ~smbus_check_error(spectral->smbus);

	if ((has_error & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
		d = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_READ_REG, DEVICE_SLAVE_ADDRESS);
	}

	counter = 0;
	while(counter < 3) {
		has_error = ~smbus_check_error(spectral->smbus);
		// Why leave when status == 0?
		HAL_Delay(5); //delay for 5 ms
		++counter;
	}
	if ((has_error & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
		*error = has_error;
	}

	smbus_write_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_WRITE_REG, v_reg, DEVICE_SLAVE_ADDRESS);
	counter = 0;
	while(counter < 3) {
		has_error = ~smbus_check_error(spectral->smbus);
		HAL_Delay(5); //delay for 5 ms
		++counter;
	}

	if ((has_error & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
		*error = has_error;
	}
	d = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_READ_REG, DEVICE_SLAVE_ADDRESS);
	return d;
}
