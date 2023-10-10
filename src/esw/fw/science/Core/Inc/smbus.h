#pragma once

#include "stm32g4xx_hal.h"
#include "stdbool.h"  // for bools
#include "stdint.h"  // for uint types
#include "stdlib.h"  // for malloc
#include "string.h"  // for memset

// An I2C Device
typedef struct
{
    I2C_HandleTypeDef *i2c;
    HAL_StatusTypeDef ret; // used to check HAL status
    uint8_t buf[30]; // can re-adjust later
} SMBus;


// REQUIRES: hi2c is the i2c channel,
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created SMBus object
SMBus *new_smbus(I2C_HandleTypeDef *hi2c);


// REQUIRES: smbus is an SMBus object
// MODIFIES: nothing
// EFFECTS: Checks if smbus->ret is HAL_OK.
// If not HAL_OK, then reset the I2C smbus
int smbus_check_error(SMBus *smbus);


// REQUIRES: smbus is an SMBus object
// MODIFIES: nothing
// EFFECTS: Deinitializes and initializes the I2C bus.
void smbus_reset(SMBus *smbus);


// REQUIRES: smbus is an SMBus object
// and reg is the command/register being read.
// MODIFIES: nothing
// EFFECTS: Reads one byte from the register.
long smbus_read_byte_data(SMBus *smbus, char reg, uint8_t device_address);


// REQUIRES: smbus is an SMBus object,
// reg is the command/register being written to,
// and data is the data being written to the register.
// MODIFIES: nothing
// EFFECTS: Writes one byte to the register.
void smbus_write_byte_data(SMBus *smbus, char reg, uint8_t data, uint8_t device_address);



