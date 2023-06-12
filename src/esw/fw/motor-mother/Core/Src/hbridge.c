
#include "hbridge.h"


HBridge *new_hbridge(bool _valid, TIM_HandleTypeDef *_timer, uint32_t _channel, uint32_t *_out_register, uint32_t *_ARR, Pin *fwd,
                     Pin *bwd) {
    HBridge *hbr = (HBridge *) malloc(sizeof(HBridge));
    hbr->valid = _valid;
    hbr->timer = _timer;
    hbr->channel = _channel;
    hbr->out_register = _out_register;
    hbr->ARR = _ARR;
    hbr->forward_pin = fwd;
    hbr->backward_pin = bwd;
    hbr->target_duty_cycle = 0;

    return hbr;
}

void init_hbridge(HBridge *hbridge, float duty_cycle, bool direction_is_forward) {
    HAL_TIM_PWM_Start(hbridge->timer, hbridge->channel);
    change_hbridge_pwm(hbridge, duty_cycle);
    change_hbridge_dir_val(hbridge, direction_is_forward);
}

void change_hbridge_pwm(HBridge *hbridge, float duty_cycle) {

    // validate input duty cycle
    if (duty_cycle < 0.0) {
        duty_cycle = 0.0;
    } else if (duty_cycle > 1.0) {
        duty_cycle = 1.0;
    }

    hbridge->target_duty_cycle = duty_cycle * (float) (*hbridge->ARR+1);

    *(hbridge->out_register) = hbridge->target_duty_cycle;

}

void change_hbridge_dir_val(HBridge *hbridge, bool val) {
    if (val) {
		write_pin_value(hbridge->backward_pin, 0);
    	write_pin_value(hbridge->forward_pin, 1);
    } else {
    	write_pin_value(hbridge->forward_pin, 0);
    	write_pin_value(hbridge->backward_pin, 1);
    }

}
