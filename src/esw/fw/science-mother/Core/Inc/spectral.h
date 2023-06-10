#pragma once

#include "stm32f1xx_hal.h"
#include "smbus.h"	// for SMBus
#include "stdint.h" // for uint types
#include "stdlib.h" // for malloc

#define SPECTRAL_DEVICES 1
#define SPECTRAL_CHANNELS 6

enum {
	DEV_SEL = 0x4F,

	I2C_AS72XX_SLAVE_STATUS_REG = 0x00,
	I2C_AS72XX_SLAVE_WRITE_REG = 0x01,
	I2C_AS72XX_SLAVE_READ_REG = 0x02,
	I2C_AS72XX_SLAVE_TX_VALID = 0x02,
	I2C_AS72XX_SLAVE_RX_VALID = 0x01,

	DEVICE_SLAVE_ADDRESS_READ = 0x93,
	DEVICE_SLAVE_ADDRESS_WRITE = 0x92,
	DEVICE_SLAVE_ADDRESS = 0x49,

	// registers for spectral sensor
	RAW_VALUE_RGA_HIGH = 0x08,
	RAW_VALUE_RGA_LOW = 0x09,

	RAW_VALUE_SHB_HIGH = 0x0A,
	RAW_VALUE_SHB_LOW = 0x0B,

	RAW_VALUE_TIC_HIGH = 0x0C,
	RAW_VALUE_TIC_LOW = 0x0D,

	RAW_VALUE_UJD_HIGH = 0x0E,
	RAW_VALUE_UJD_LOW = 0x0F,

	RAW_VALUE_VKE_HIGH = 0x10,
	RAW_VALUE_VKE_LOW = 0x11,

	RAW_VALUE_WLF_HIGH = 0x12,
	RAW_VALUE_WLF_LOW = 0x13,
	AS7262_V_CAL = 0x14,
	AS7262_B_CAL = 0x18,
	AS7262_G_CAL = 0x1C,
	AS7262_Y_CAL = 0x20,
	AS7262_O_CAL = 0x24,
	AS7262_R_CAL = 0x28,

	CONTROL_SET_UP = 0x04,
	INT_TIME = 0x05
};

// AS7262 Spectral sensor
typedef struct
{
	SMBus *smbus;
	uint16_t channel_data[SPECTRAL_CHANNELS];
} Spectral;

// REQUIRES: SMBus declared by user
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Spectral object
Spectral *new_spectral(SMBus *smbus);

// REQUIRES: spectral is a Spectral object
// MODIFIES: nothing
// EFFECTS: Initializes the spectral device
void initialize_spectral(Spectral *spectral);

// REQUIRES: spectral is an object
// MODIFIES: spectral.channels array
// EFFECTS: Updates values of spectral struct's channels array with data from spectral sensor
void update_spectral_all_channel_data(Spectral *spectral);

// REQUIRES: spectral is an object and 0 <= channel < 6
// MODIFIES: spectral.channels array
// EFFECTS: Updates values of spectral struct's channels array with data from spectral sensor
void update_spectral_channel_data(Spectral *spectral, uint8_t channel, uint8_t *error_flag);

// REQUIRES: spectral is an object and 0 <= channel < 6
// MODIFIES: nothing
// EFFECTS: Returns the spectral data of a particular channel
uint16_t get_spectral_channel_data(Spectral *spectral, uint8_t channel);

// REQUIRES: spectral is a Spectral object,
// v_reg is the virtual register,
// and data is the data to pass
// MODIFIES: nothing
// EFFECTS: Writes to the virtual register as explained in page 18-20 of the datasheet
void virtual_write_spectral(Spectral *spectral, uint8_t v_reg, uint8_t data);

// REQUIRES: spectral is a Spectral object and
// v_reg is the virtual register
// MODIFIES: nothing
// EFFECTS: Returns the value read from the virtual register
// as explained in page 18-20 of the datasheet
uint8_t virtual_read_spectral(Spectral *spectral, uint8_t v_reg, uint8_t *error);
