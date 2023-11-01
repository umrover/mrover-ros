#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f3xx_hal.h"

#ifndef SMBUS_H_
#define SMBUS_H_

#define TRUE 1
#define FALSE 0

typedef struct {
    I2C_HandleTypeDef *i2c;
    HAL_StatusTypeDef ret;
    uint8_t buf[30];
    uint8_t DMA;
} SMBus;

SMBus *new_smbus(I2C_HandleTypeDef *hi2c);

long read_byte(SMBus *smbus, uint8_t addr);

void write_byte(SMBus *smbus, uint8_t addr, uint8_t data);

long read_byte_data(SMBus *smbus, uint8_t addr, char cmd);

void write_byte_data(SMBus *smbus, uint8_t addr, char cmd, uint8_t data);

long read_word_data(SMBus *smbus, uint8_t addr, char cmd);

void write_word_data(SMBus *smbus, uint8_t addr, char cmd, uint16_t data);

int _check_error(SMBus *smbus);

void reset(SMBus *smbus);

void disable_DMA(SMBus *smbus);

void del_smbus(SMBus *smbus);

#endif
