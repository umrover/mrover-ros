#pragma once

#include <stdlib.h>
#include <math.h>
#include "stm32f1xx_hal.h"

#include "pin.h"

typedef struct {
    Pin *pin;
    uint8_t enabled;
    uint8_t is_pressed;
    uint8_t valid;
    uint8_t active_high;
    int32_t associated_count;
} LimitSwitch;

LimitSwitch *new_limit_switch(uint8_t _valid, Pin *_pin);

void update_limit_switch(LimitSwitch *limit_switch);
