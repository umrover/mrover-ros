#include "auton_led.h"

// REQUIRES: _red_led, _green_led, and _blue_led are PinData objects
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created AutonLED object
AutonLED *new_auton_led(PinData *_red_led, PinData *_green_led, PinData *_blue_led) {
    AutonLED *auton_led = (AutonLED*) malloc(sizeof(AutonLED));
    auton_led->red_led = _red_led;
    auton_led->green_led = _green_led;
    auton_led->blue_led = _blue_led;
    auton_led->state = AUTON_LED_OFF_STATE;
    auton_led->green_state_time_elapsed_ms = 0;
    return auton_led;
}

// REQUIRES: auton_led is an AutonLED object and 0 <= state <= 3
// MODIFIES: state
// EFFECTS: Changes the auton_led state
void change_auton_led_state(AutonLED *auton_led, int state) {
	if (state < 0 || state > 3) {
		// return if 0 <= state <= 3 is not true
		return;
	}

	if (auton_led->state != state) {
		if (auton_led->state == AUTON_LED_RED_STATE) {
			set_pin_value(auton_led->red_led, false);
		}
		else if (auton_led->state == AUTON_LED_BLINKING_GREEN_STATE) {
			set_pin_value(auton_led->green_led, false);
		}
		else if (auton_led->state == AUTON_LED_BLUE_STATE) {
			set_pin_value(auton_led->blue_led, false);
		}

		auton_led->state = state;
		if (state == AUTON_LED_RED_STATE) {
			set_pin_value(auton_led->red_led, true);
		}
		else if (state == AUTON_LED_BLINKING_GREEN_STATE) {
			set_pin_value(auton_led->green_led, true);
			auton_led->green_state_time_elapsed_ms = 0;
		}
		else if (state == AUTON_LED_BLUE_STATE) {
			set_pin_value(auton_led->blue_led, true);
		}
	}
}

// REQUIRES: auton_led is an AutonLED object.
// This function is expected to be called every 1 ms
// EFFECTS: Changes the auton_led colors.
// This is specifically so the blinking green lights work properly
void update_auton_led_state(AutonLED *auton_led) {
	if (auton_led->state == AUTON_LED_BLINKING_GREEN_STATE) {
		++auton_led->green_state_time_elapsed_ms;
		if (auton_led->green_state_time_elapsed_ms % AUTON_LED_GREEN_STATE_PERIOD_MS == 0) {
			set_pin_value(auton_led->green_led, true);
		}
		else if (auton_led->green_state_time_elapsed_ms % AUTON_LED_GREEN_STATE_PERIOD_MS == AUTON_LED_GREEN_STATE_ON_DURATION_MS) {
			set_pin_value(auton_led->green_led, false);
		}
	}
}
