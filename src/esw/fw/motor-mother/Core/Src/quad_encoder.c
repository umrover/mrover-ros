#include "quad_encoder.h"

QuadEncoder *new_quad_encoder(bool _valid, TIM_HandleTypeDef *_htim, TIM_TypeDef *_tim) {
    QuadEncoder *quad_encoder = (QuadEncoder *) malloc(sizeof(QuadEncoder));
    quad_encoder->valid = _valid;
    quad_encoder->tim = _tim;
    quad_encoder->htim = _htim;
    quad_encoder->counts = 0;
    quad_encoder->counts_raw_prev = 0;
    quad_encoder->counts_raw_now = 0;
    return quad_encoder;
}

void init_quad_encoder(QuadEncoder* quad_encoder) {
	set_encoder_counts(quad_encoder, 0);
}

void update_quad_encoder(QuadEncoder* quad_encoder) {
	quad_encoder->counts_raw_now = quad_encoder->tim->CNT;
    quad_encoder->counts += (int16_t)(quad_encoder->counts_raw_now - quad_encoder->counts_raw_prev);
    quad_encoder->counts_raw_prev = quad_encoder->counts_raw_now;
}

void set_encoder_counts(QuadEncoder *quad_encoder, int32_t counts) {
	quad_encoder->counts = 0;
	quad_encoder->counts_raw_prev = quad_encoder->tim->CNT;
	quad_encoder->counts_raw_now = quad_encoder->tim->CNT;
}
