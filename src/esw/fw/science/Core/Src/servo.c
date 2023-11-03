#include "servo.h"

// REQUIRES: timer is a valid timer, channel & output are a valid channel number and output pointer
// MODIFIES: Nothing
// EFFECTS: Creates and returns a pointer to a servo object with the specified parameters
Servo* new_servo(TIM_HandleTypeDef* timer, uint32_t channel, uint32_t *output) {

	Servo* servo = (Servo*) malloc(sizeof(Servo));
	servo->timer = timer;
	servo->channel = channel;
	servo->output = output;
	return servo;

}

// REQUIRES: servo is a pointer to a valid Servo object, angle is a valid angle in degrees
// MODIFIES: servo
// EFFECTS: sets the servo angle of the provided servo object to be angle (degrees)
void set_servo_angle(Servo* servo, double angle) {
	*(servo->output) = (uint32_t) ((ANGLE_CONVERSION_RATIO * angle) + ANGLE_OFFSET);
}

// REQUIRES: servo is a pointer to a valid Servo object, angle is a valid angle in degrees
// MODIFIES: servo
// EFFECTS: initializes the servo object pointed to to be set at the given angle using math
void initialize_servo(Servo* servo, double angle) {
	HAL_TIM_PWM_Start(servo->timer, servo->channel);
	set_servo_angle(servo, angle);
}