#pragma once

#include "stm32g4xx_hal.h"

#include <stdlib.h>

#define ANGLE_CONVERSION_RATIO (5.0/9)
#define ANGLE_OFFSET 150.0 


typedef struct{

	TIM_HandleTypeDef* timer;
	uint32_t channel;
	uint32_t *output;

} Servo;

// REQUIRES: timer is a valid timer, channel & output are a valid channel number and output pointer
// MODIFIES: Nothing
// EFFECTS: Creates and returns a pointer to a servo object with the specified parameters
Servo* new_servo(TIM_HandleTypeDef* timer, uint32_t channel, uint32_t *output);

// REQUIRES: servo is a pointer to a valid Servo object, angle is a valid angle in degrees
// MODIFIES: servo
// EFFECTS: initializes the servo object pointed to to be set at the given angle using math
void initialize_servo(Servo* servo, double angle);

// REQUIRES: servo is a pointer to a valid Servo object, angle is a valid angle in degrees
// MODIFIES: servo
// EFFECTS: sets the servo angle of the provided servo object to be angle (degrees)
void set_servo_angle(Servo* servo, double angle);