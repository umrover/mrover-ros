#pragma once

#include <closed_loop_control.h>
#include <stdlib.h>
#include <math.h>
#include "stdbool.h"
#include "stm32f1xx_hal.h"

#include "pin.h"
#include "hbridge.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    TIM_TypeDef *tim;

    bool valid;
    int32_t counts;
    int16_t counts_raw_prev;
    int16_t counts_raw_now;
} QuadEncoder;

QuadEncoder *new_quad_encoder(bool _valid, TIM_HandleTypeDef *_htim, TIM_TypeDef *_tim);

void init_quad_encoder(QuadEncoder* quad_encoder);

void update_quad_encoder(QuadEncoder* quad_encoder);

void set_encoder_counts(QuadEncoder *quad_encoder, int32_t counts);
