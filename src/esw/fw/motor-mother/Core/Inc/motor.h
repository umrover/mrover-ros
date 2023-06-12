#pragma once

#include <closed_loop_control.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f1xx_hal.h"

#include "limit_switch.h"
#include "pin.h"
#include "hbridge.h"
#include "quad_encoder.h"
#include "abs_enc_reading.h"

#define MOTORS_WATCHDOG_TIMEOUT 443

typedef struct {
    HBridge *hbridge;
    LimitSwitch *limit_switch_a;
    LimitSwitch *limit_switch_b;
    QuadEncoder *encoder;
    AbsEncoder *abs_encoder;
    ClosedLoopControl *control;

    uint8_t valid;
    uint8_t using_open_loop_control;
    float output_pwm; // USE FOR PWM! Should be between -max_pwm and max_pwm
    float max_pwm;  // A configuration value! Should be between 0 and 1
    float desired_speed; // Do not use raw value for PWM! Should be between -1 and 1
    int32_t desired_counts;
    uint8_t is_calibrated;
    uint8_t limit_a_is_forward;
    uint32_t ms_since_last_commanded;
} Motor;

Motor *new_motor(bool _valid, HBridge *_hbridge, LimitSwitch *_limit_switch_a, LimitSwitch *_limit_switch_b, QuadEncoder *_encoder, AbsEncoder *_abs_encoder, ClosedLoopControl *_control);

void init_motor(Motor *motor, float speed);

void tick_motor(Motor *motor);

void update_motor_target(Motor *motor);

void set_motor_speed(Motor *motor, float speed);

void update_motor_speed(Motor *motor);

void update_motor_limits(Motor *motor);

void move_motor_to_target(Motor *motor);

void update_motor_limit_switches(Motor *motor);

void refresh_motor_absolute_encoder_value(Motor *motor);
