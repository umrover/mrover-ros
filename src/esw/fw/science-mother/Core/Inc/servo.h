#pragma once

#include "stm32f1xx_hal.h"
#include <stdlib.h>

// SG90 Servo Motor
// Duty cycle should be 20 ms long. Should be high for between 1-2 ms.
// See http://www.ee.ic.ac.uk/pcheung/teaching/DE1_EE/stores/sg90_datasheet.pdf
typedef struct {
	TIM_HandleTypeDef *timer;
	uint32_t channel;
	uint32_t *out_channel;
} Servo;

// REQUIRES: timer is the timer and channel is the channel and out channel is the CCR
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Servo object
Servo *new_servo(TIM_HandleTypeDef *_timer, uint32_t _channel, uint32_t *_out_channel);

// REQUIRES: servo is a Servo object and initial angle is the initial angle
// MODIFIES: nothing
// EFFECTS: Initializes the servo by configuring the timer settings
void initialize_servo(Servo* servo, int16_t initial_angle);

// REQUIRES: servo is a Servo object and angle is desired angle in degrees
// and 0 <= angle <= 180
// NOTE: the servo has a 120 degree range, so input will need to be scaled accordingly
// MODIFIES: nothing
// EFFECTS: Sets the servo angle to an absolute position
void set_servo_angle(Servo *servo, int16_t angle);
