#pragma once

#include <stdlib.h>
#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "pin.h"

typedef struct {
	bool valid;
    TIM_HandleTypeDef *timer;
    uint32_t channel;
    uint32_t *out_register;
    uint32_t *ARR;
    Pin *forward_pin;
    Pin *backward_pin;
    uint32_t target_duty_cycle;
} HBridge;


// Returns pointer to a new hbridge object
HBridge *new_hbridge(
		bool _valid,
		TIM_HandleTypeDef *_timer,
		uint32_t _channel,
		uint32_t *_out_register,
		uint32_t *_ARR,
		Pin *fwd,
		Pin *bwd);

// Initialize timer settings
void init_hbridge(HBridge *hbridge, float duty_cycle, bool direction_is_forward);

// Requires a signal of between 0 and 1 for duty cycle
// Calculates high/low pulse durations and sends to hbridge
void change_hbridge_pwm(HBridge *hbridge, float duty_cycle);

void change_hbridge_dir_val(HBridge *hbridge, bool val);

