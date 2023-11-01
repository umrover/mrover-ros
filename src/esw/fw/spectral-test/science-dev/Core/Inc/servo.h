#ifndef SERVO_H_
#define SERVO_H_

// SG90 Servo Motor
typedef struct {
	TIM_HandleTypeDef *timer;
	uint8_t channel;
} Servo;

// REQUIRES: timer is the timer and channel is the channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Servo object
Servo *new_servo(TIM_HandleTypeDef *_timer, uint8_t _channel);

// REQUIRES: servo is a Servo object
// MODIFIES: nothing
// EFFECTS: Initializes the servo by configuring the timer settings
void initialize_servo(Servo* servo);

// REQUIRES: servo is a Servo object and angle is desired angle in degrees
// and 0 <= angle <= 180
// MODIFIES: nothing
// EFFECTS: Sets the servo angle to an absolute position
uint32_t set_servo_angle(Servo *servo, uint16_t angle);

#endif

//#endif
