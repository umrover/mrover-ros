#include "smbus.h"

// REQUIRES: hi2c is the i2c channel,
// huart is the uart channel or is NULL,
// and _dma tells if DMA is enabled
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created SMBus object
SMBus *new_smbus(
    I2C_HandleTypeDef *hi2c,
    UART_HandleTypeDef *huart,
    bool _dma)
{
    SMBus *smbus = malloc(sizeof(SMBus));
    smbus->i2c = hi2c;
    smbus->uart = huart;
    smbus->ret = HAL_OK;
    memset(smbus->buf, 0, sizeof(smbus->buf));
    smbus->DMA = _dma;
}

// REQUIRES: smbus is an SMBus object
// MODIFIES: nothing
// EFFECTS: Checks if smbus->ret is HAL_OK.
// If not HAL_OK, then reset the I2C smbus
// and if smbus->uart is not NULL, then
// print out an error message.
int smbus_check_error(SMBus *smbus)
{
    if (smbus->ret == HAL_OK)
    {
        return true;
    }

    smbus_reset(smbus);
    HAL_Delay(10);
    return false;
}

// REQUIRES: smbus is an SMBus object
// and reg is the command/register being read.
// MODIFIES: nothing
// EFFECTS: Reads one byte from the register.
long smbus_read_byte_data(
    SMBus *smbus,
    char reg,
	uint8_t device_address)
{
    smbus->buf[0] = reg;
    if (!smbus->DMA)
    {
        smbus->ret = HAL_I2C_Master_Transmit(
            smbus->i2c,
            device_address << 1,
            smbus->buf,
            1,
            50);
        smbus_check_error(smbus);
        smbus->ret = HAL_I2C_Master_Receive(
            smbus->i2c,
            (device_address << 1) | 1,
            smbus->buf,
            1,
            50);
    }
    else
    {
        smbus->ret = HAL_I2C_Master_Transmit_DMA(
            smbus->i2c,
            device_address << 1,
            smbus->buf,
            1);
        smbus_check_error(smbus);
        smbus->ret = HAL_I2C_Master_Receive_DMA(
            smbus->i2c,
            (device_address << 1) | 1,
            smbus->buf,
            1);
    }
    HAL_Delay(10);
    smbus_check_error(smbus);
    return smbus->buf[0];
}

// REQUIRES: smbus is an SMBus object
// and reg is the command/register being read.
// MODIFIES: nothing
// EFFECTS: Reads two bytes from the register.
long smbus_read_word_data(
    SMBus *smbus,
    char reg,
	uint8_t device_address)
{
    smbus->buf[0] = reg;
    if (!smbus->DMA)
    {
        smbus->ret = HAL_I2C_Master_Transmit(
            smbus->i2c,
            device_address << 1,
            smbus->buf,
            1,
            50);
        smbus_check_error(smbus);
        smbus->ret = HAL_I2C_Master_Receive(
            smbus->i2c,
            (device_address << 1) | 1,
            smbus->buf,
            2,
            50);
    }
    else
    {
        smbus->ret = HAL_I2C_Master_Transmit_DMA(
            smbus->i2c,
            device_address << 1,
            smbus->buf,
            1);
        smbus_check_error(smbus);
        smbus->ret = HAL_I2C_Master_Receive_DMA(
            smbus->i2c,
            (device_address << 1) | 1,
            smbus->buf,
            2);
    }
    HAL_Delay(50);
    long data = smbus->buf[0] | (smbus->buf[1] << 8);
    return data;
}

// REQUIRES: smbus is an SMBus object
// MODIFIES: nothing
// EFFECTS: Deinitializes and initializes the I2C bus.
void smbus_reset(SMBus *smbus)
{
    HAL_I2C_DeInit(smbus->i2c);
    HAL_I2C_Init(smbus->i2c);
}

// REQUIRES: smbus is an SMBus object,
// reg is the command/register being written to,
// and data is the data being written to the register.
// MODIFIES: nothing
// EFFECTS: Writes one byte to the register.
void smbus_write_byte_data(
    SMBus *smbus,
    char reg,
    uint8_t data,
	uint8_t device_address)
{
    smbus->buf[0] = reg;
    smbus->buf[1] = data;

    if (!smbus->DMA)
    {
        smbus->ret = HAL_I2C_Master_Transmit(
            smbus->i2c,
            device_address << 1,
            smbus->buf,
            2,
            50);
    }
    else
    {
        smbus->ret = HAL_I2C_Master_Transmit_DMA(
            smbus->i2c,
            device_address << 1,
            smbus->buf,
            2);
    }
    HAL_Delay(10);
    smbus_check_error(smbus);
}

// REQUIRES: smbus is an SMBus object,
// reg is the command/register being written to,
// and data is the data being written to the register.
// MODIFIES: nothing
// EFFECTS: Writes two bytes to the register.
void smbus_write_word_data(
    SMBus *smbus,
    char reg,
    uint16_t data,
	uint8_t device_address)
{
    smbus->buf[0] = reg;
    smbus->buf[1] = data & 0xFF;
    smbus->buf[2] = (data & 0xFF00) >> 8;

    if (!smbus->DMA)
    {
        smbus->ret = HAL_I2C_Master_Transmit(
            smbus->i2c,
            device_address << 1,
            smbus->buf,
            3,
            50);
    }
    else
    {
        smbus->ret = HAL_I2C_Master_Transmit_DMA(
            smbus->i2c,
            device_address << 1,
            smbus->buf,
            3);
    }
    HAL_Delay(50);
    smbus_check_error(smbus);
}
