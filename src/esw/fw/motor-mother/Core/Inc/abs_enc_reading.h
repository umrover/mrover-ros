/*
 * abs_enc_read.h
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

// assume using AS5048B - 14 bit

#ifndef INC_ABS_ENC_READING_H_
#define INC_ABS_ENC_READING_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include "stdint.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


#define RAW_TO_180_DEGREES_CONVERSION_FACTOR 8192.0f
#define RAW_TO_RADIANS_CONVERSION_FACTOR 2607.594587f;
#define TRUE 1
#define FALSE 0

enum {
	device_slave_address_none_power = 0x40,
	device_slave_address_a1_power = 0x41,
	device_slave_address_a2_power = 0x42,
	device_slave_address_both_power = 0x43,
};

typedef struct {
    I2C_HandleTypeDef *i2c;
    HAL_StatusTypeDef ret;
    uint8_t buf[30];
    uint8_t DMA;
} SMBus;

typedef struct {
	bool valid;
	int address;
	float angle_rad;
	SMBus* i2cBus;
} AbsEncoder;

SMBus *new_smbus(I2C_HandleTypeDef *hi2c);

long read_word_data(SMBus *smbus, uint8_t addr, char cmd);

void disable_DMA(SMBus *smbus);

void del_smbus(SMBus *smbus);

// A1/A2 is 1 if pin connected to power, 0 if pin connected to ground
AbsEncoder* new_abs_encoder(bool _valid, SMBus* i2cBus, uint8_t A1, uint8_t A2);

int read_raw_angle(AbsEncoder* abs_encoder);

void refresh_angle_radians(AbsEncoder* encoder);

void del_encoder(AbsEncoder*);

AbsEncoder* abs_encoder_init(bool valid, I2C_HandleTypeDef* abs_encoder_handle, uint8_t A1, uint8_t A2);

#endif /* INC_ABS_ENC_READING_H_ */
