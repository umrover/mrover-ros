#include "smbus.h"

// REQUIRES: hi2c is the i2c channel,
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created SMBus object
SMBus *new_smbus(I2C_HandleTypeDef *hi2c){
    SMBus* smbus = malloc(sizeof(SMBus));
    smbus->i2c = hi2c;
    return smbus;
}


// REQUIRES: smbus is an SMBus object
// MODIFIES: nothing
// EFFECTS: Checks if smbus->ret is HAL_OK.
// If not HAL_OK, then reset the I2C smbus
int smbus_check_error(SMBus *smbus){
    if (smbus->ret != HAL_OK) {
        smbus_reset(smbus);
        return  0;
    }
    return 1;
}


// REQUIRES: smbus is an SMBus object
// MODIFIES: nothing
// EFFECTS: Deinitializes and initializes the I2C bus.
void smbus_reset(SMBus *smbus){
    HAL_I2C_DeInit(smbus->i2c);
    HAL_I2C_Init(smbus->i2c);
}


// REQUIRES: smbus is an SMBus object
// and reg is the command/register being read.
// MODIFIES: nothing
// EFFECTS: Reads one byte from the register.
long smbus_read_byte_data(SMBus *smbus, char reg, uint8_t device_address){
    HAL_I2C_Mem_Read(smbus->i2c, device_address, reg, I2C_MEMADD_SIZE_8BIT, smbus->buf, 1, HAL_MAX_DELAY);
    return smbus->buf[0];
}


// REQUIRES: smbus is an SMBus object,
// reg is the command/register being written to,
// and data is the data being written to the register.
// MODIFIES: nothing
// EFFECTS: Writes one byte to the register.
void smbus_write_byte_data(SMBus *smbus, char reg, uint8_t data, uint8_t device_address){
    
    HAL_I2C_Mem_Write(
        smbus->i2c, //i2c 
        device_address, //device address
        reg,  //internal memory address 
        I2C_MEMADD_SIZE_8BIT, //size of internal memory address 
        &data, //pointer to data buffer
        1, //bytes to write 
        HAL_MAX_DELAY //milliseconds to wait before timing out
    );
    
}

// Write(0)to 0x70
// sets the desination to port 0
//write(1)  to 0x70 sends data 1 to port 0