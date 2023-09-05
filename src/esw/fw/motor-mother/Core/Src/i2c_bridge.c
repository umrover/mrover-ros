/*
 * i2c_bridge.c
 *
 *  Created on: Oct 26, 2020
 *      Author: Raymond Liu
 */

#include "i2c_bridge.h"
#include "motor.h"


I2CBus i2c_bus_default = {
        UNKNOWN, //operation
        0xFF, //channel
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, //buffer
        0
};

I2CBus *new_i2c_bus(I2C_HandleTypeDef *_i2c_bus_handle) {
    I2CBus *bus = (I2CBus *) malloc(sizeof(I2CBus));
    bus->i2c_bus_handle = _i2c_bus_handle;
    bus->operation = UNKNOWN;
    bus->tick = 0x00;
    bus->motor_id = 0xFF;
    for(size_t i = 0 ; i < 32 ; ++i) {
        bus->buffer[i] = 0x00;
    }
    return bus;
}

uint8_t CH_num_receive(I2CBus *i2c_bus) {
    switch (i2c_bus->operation) {
        case OFF:
        case ON:
            return 0;
        case OPEN:
        case OPEN_PLUS:
            return 4;
        case CLOSED:
        case CLOSED_PLUS:
            return 8;
        case CONFIG_PWM:
            return 2;
        case CONFIG_K:
            return 12;
        case QUAD_ENC:
            return 0;
        case ADJUST:
            return 4;
        case ABS_ENC:
        case IS_CALIBRATED:
        	return 0;
        case ENABLE_LIMIT_A:
        case ENABLE_LIMIT_B:
        case ACTIVE_LIMIT_A:
        case ACTIVE_LIMIT_B:
        	return 1;
        case COUNTS_LIMIT_A:
        case COUNTS_LIMIT_B:
        	return 4;
        case LIMIT_A:
        case LIMIT_B:
        	return 0;
        case LIMIT_A_IS_FWD:
        	return 1;
        case LIMIT_DATA:
        	return 0;
        case UNKNOWN:
            return 0;
    }
    return 0;
}

uint8_t CH_num_send(I2CBus *i2c_bus) {
    switch (i2c_bus->operation) {
        case OFF:
        case ON:
        case OPEN:
            return 0;
        case OPEN_PLUS:
            return 4;
        case CLOSED:
            return 0;
        case CLOSED_PLUS:
            return 4;
        case CONFIG_PWM:
        case CONFIG_K:
            return 0;
        case QUAD_ENC:
            return 4;
        case ADJUST:
            return 0;
        case ABS_ENC:
            return 4;
        case IS_CALIBRATED:
        	return 1;
        case ENABLE_LIMIT_A:
        case ENABLE_LIMIT_B:
        case ACTIVE_LIMIT_A:
        case ACTIVE_LIMIT_B:
        case COUNTS_LIMIT_A:
        case COUNTS_LIMIT_B:
        	return 0;
        case LIMIT_A:
        case LIMIT_B:
        	return 1;
        case LIMIT_A_IS_FWD:
        	return 0;
        case LIMIT_DATA:
        	return 1;
        case UNKNOWN:
            return 0;
    }
    return 0;
}


void CH_process_received(I2CBus *i2c_bus, Motor *motor) {
    switch (i2c_bus->operation) {
        case OFF:
            set_motor_speed(motor, 0.0f);
            return;
        case ON:
            return;
        case OPEN:
        case OPEN_PLUS:
        	motor->ms_since_last_commanded = 0;
            motor->using_open_loop_control = 1;
            memcpy(&(motor->desired_speed), i2c_bus->buffer, 4);
            return;
        case CLOSED:
        case CLOSED_PLUS:
        	motor->ms_since_last_commanded = 0;
            motor->using_open_loop_control = 0;
            memcpy(&(motor->control->kF), i2c_bus->buffer, 4);
            memcpy(&(motor->desired_counts), i2c_bus->buffer + 4, 4);
            return;
        case CONFIG_PWM: {
            int max = 0;
            memcpy(&(max), i2c_bus->buffer, 2);
            motor->max_pwm = ((float) max) / 100.0f;
            return;
        }
        case CONFIG_K:
            memcpy(&(motor->control->kP), i2c_bus->buffer, 4);
            memcpy(&(motor->control->kI), i2c_bus->buffer + 4, 4);
            memcpy(&(motor->control->kD), i2c_bus->buffer + 8, 4);
            return;
        case QUAD_ENC:
            return;
        case ADJUST:
            memcpy(&(motor->encoder->counts), i2c_bus->buffer, 4);
	    motor->is_calibrated = 1;
            return;
        case ABS_ENC:
        case IS_CALIBRATED:
            return;
        case ENABLE_LIMIT_A:
        	motor->is_calibrated = 0;
        	memcpy(&(motor->limit_switch_a->enabled), i2c_bus->buffer, 1);
        	return;
        case ENABLE_LIMIT_B:
        	motor->is_calibrated = 0;
        	memcpy(&(motor->limit_switch_b->enabled), i2c_bus->buffer, 1);
        	return;
        case ACTIVE_LIMIT_A:
        	memcpy(&(motor->limit_switch_a->active_high), i2c_bus->buffer, 1);
        	return;
        case ACTIVE_LIMIT_B:
        	memcpy(&(motor->limit_switch_b->active_high), i2c_bus->buffer, 1);
        	return;
        case COUNTS_LIMIT_A:
        	memcpy(&(motor->limit_switch_a->associated_count), i2c_bus->buffer, 4);
        	return;
        case COUNTS_LIMIT_B:
        	memcpy(&(motor->limit_switch_b->associated_count), i2c_bus->buffer, 4);
        	return;
        case LIMIT_A:
        case LIMIT_B:
        	return;
        case LIMIT_A_IS_FWD:
        	memcpy(&(motor->limit_a_is_forward), i2c_bus->buffer, 1);
        	return;
        case LIMIT_DATA:
        	return;
        case UNKNOWN:
            return;
    }
}

void CH_prepare_send(I2CBus *i2c_bus, Motor *motor) {
    switch (i2c_bus->operation) {
        case OFF:
        case ON:
        case OPEN:
            return;
        case OPEN_PLUS:
            memcpy(i2c_bus->buffer, &(motor->encoder->counts), 4);
            return;
        case CLOSED:
            return;
        case CLOSED_PLUS:
            memcpy(i2c_bus->buffer, &(motor->encoder->counts), 4);
            return;
        case CONFIG_PWM:
        case CONFIG_K:
            return;
        case QUAD_ENC:
            memcpy(i2c_bus->buffer, &(motor->encoder->counts), 4);
            return;
        case ADJUST:
            return;
        case ABS_ENC:
        	memcpy(i2c_bus->buffer, &(motor->abs_encoder->angle_rad), 4);
        	return;
        case IS_CALIBRATED:
            memcpy(i2c_bus->buffer, &(motor->is_calibrated), 1);
            return;
        case ENABLE_LIMIT_A:
        case ENABLE_LIMIT_B:
        case ACTIVE_LIMIT_A:
        case ACTIVE_LIMIT_B:
        case COUNTS_LIMIT_A:
        case COUNTS_LIMIT_B:
        	return;
        case LIMIT_A:
        	memcpy(i2c_bus->buffer, &(motor->limit_switch_a->is_pressed), 1);
			return;
        case LIMIT_B:
        	memcpy(i2c_bus->buffer, &(motor->limit_switch_b->is_pressed), 1);
        	return;
        case LIMIT_A_IS_FWD:
        	return;
        case LIMIT_DATA: ;
        	// 0th (least significant) bit is a boolean (bit) that tells if the motor has been calibrated or not.
			// 1st bit is a boolean (bit) that is the state of limit switch a.
			// 2nd bit is a boolean (bit) that is the state of limit switch b.
			uint8_t limit_data_val = 0;
			limit_data_val |= (motor->is_calibrated & 0x1);
			limit_data_val |= ((motor->limit_switch_a->is_pressed & 0x1) << 1);
			limit_data_val |= ((motor->limit_switch_b->is_pressed & 0x1) << 2);
			memcpy(i2c_bus->buffer, &limit_data_val, 1);
        case UNKNOWN:
            return;
    }
}

void CH_reset(I2CBus *i2c_bus) {
    i2c_bus->operation = UNKNOWN;
    // The reason why we have a DeInit/Init is to clear the errors of the I2C bus by resetting it.
	// If we don't do this, then it's possible for the I2C bus to have errors and never receive new messages.
	HAL_I2C_DeInit(i2c_bus->i2c_bus_handle);
    HAL_I2C_Init(i2c_bus->i2c_bus_handle);
	HAL_I2C_EnableListen_IT(i2c_bus->i2c_bus_handle);
}

void CH_tick(I2CBus *i2c_bus) {
    i2c_bus->tick += 1;
    if (i2c_bus->tick >= I2C_WATCHDOG_TIMEOUT) {
        i2c_bus->tick = 0;
        CH_reset(i2c_bus);
    }
}
