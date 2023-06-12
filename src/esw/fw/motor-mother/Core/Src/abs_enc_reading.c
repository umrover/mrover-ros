/*
 * abs_enc_read.c
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#include "abs_enc_reading.h"

SMBus *new_smbus(I2C_HandleTypeDef *hi2c) {
    SMBus *smbus = malloc(sizeof(SMBus));
    smbus->i2c = hi2c;
    memset(smbus->buf, 0, sizeof(smbus->buf));
    return smbus;
}

long read_word_data(SMBus *smbus, uint8_t addr, char cmd) {
    smbus->buf[0] = cmd;
    smbus->ret = HAL_I2C_Master_Transmit(smbus->i2c, addr << 1, smbus->buf, 1, 500);

    //reads from address sent above
    smbus->ret = HAL_I2C_Master_Receive(smbus->i2c, (addr << 1) | 1, smbus->buf, 2, 500);

    long data = smbus->buf[0] | (smbus->buf[1] << 8);
    if (smbus->ret != HAL_OK)
    {
    	HAL_I2C_DeInit(smbus->i2c);
    	HAL_Delay(5);
    	HAL_I2C_Init(smbus->i2c);
    	data = 0;
    }

    return data;
}

void del_smbus(SMBus *smbus) {
	free(smbus->buf);
	free(smbus);
}

// A1/A2 is 1 if pin connected to power, 0 if pin connected to ground
AbsEncoder* new_abs_encoder(bool _valid, SMBus* i2cBus, uint8_t A1, uint8_t A2){
	AbsEncoder* abs_encoder = (AbsEncoder*) malloc(sizeof(AbsEncoder));
    if (A1 && A2) abs_encoder->address = device_slave_address_both_power;
    else if (A1) abs_encoder->address = device_slave_address_a1_power;
    else if (A2) abs_encoder->address = device_slave_address_a2_power;
    else abs_encoder->address = device_slave_address_none_power;
    abs_encoder->valid = _valid;
    abs_encoder->i2cBus = i2cBus;
    abs_encoder->angle_rad = 0;
    return abs_encoder;
}

int read_raw_angle(AbsEncoder* abs_encoder) {
	int raw_data = read_word_data(abs_encoder->i2cBus, abs_encoder->address, 0xFF);
	int angle_left = ( raw_data >> 8 ) & 0xFF; // 0xFE
	int angle_right = raw_data & 0xFF; // 0xFF
	int angle_left_modified = angle_left & 0x3F;
	int angle_raw = (angle_right << 6) | angle_left_modified;
    return angle_raw;
}

long read_byte_data(SMBus *smbus, uint8_t addr, char cmd) {
    //transmits the address to read from
    smbus->buf[0] = cmd;
	smbus->ret = HAL_I2C_Master_Transmit(smbus->i2c, addr << 1, smbus->buf, 1, 50);

    //reads from address sent above
	smbus->ret = HAL_I2C_Master_Receive(smbus->i2c, (addr << 1) | 1, smbus->buf, 1, 50);

    return smbus->buf[0];
}

void refresh_angle_radians(AbsEncoder* encoder) {
	int angle_raw = read_raw_angle(encoder);
	float radians = (float)angle_raw / RAW_TO_RADIANS_CONVERSION_FACTOR;
	encoder->angle_rad = radians;
}

void del_encoder(AbsEncoder* abs_encoder){
    free(abs_encoder);
}

AbsEncoder* abs_encoder_init(bool valid, I2C_HandleTypeDef* abs_encoder_handle, uint8_t A1, uint8_t A2){
	SMBus* i2cBus = new_smbus(abs_encoder_handle);
	return new_abs_encoder(valid, i2cBus, A1, A2);
}
