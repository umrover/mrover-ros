#include "limit_switch.h"


LimitSwitch *new_limit_switch(uint8_t _valid, Pin *_pin) {
	LimitSwitch *limit_switch = (LimitSwitch *) malloc(sizeof(LimitSwitch));
	limit_switch->valid = _valid;
	limit_switch->pin = _pin;
	limit_switch->enabled = 0;
	limit_switch->is_pressed = 0;
	limit_switch->associated_count = 0;
	limit_switch->active_high = 0;
	return limit_switch;
}

void update_limit_switch(LimitSwitch *limit_switch) {
	// This suggests active low
	if (limit_switch->valid && limit_switch->enabled) {
		limit_switch->is_pressed = (limit_switch->active_high == read_pin_value(limit_switch->pin));
	}
	else {
		limit_switch->is_pressed = 0;
	}
}
