#pragma once

#include <stdbool.h>

#include "pin_data.h"

#define AUTON_LED_RED_STATE 0
#define AUTON_LED_BLINKING_GREEN_STATE 1
#define AUTON_LED_BLUE_STATE 2
#define AUTON_LED_OFF_STATE 3
#define AUTON_LED_GREEN_STATE_ON_DURATION_MS 1000
#define AUTON_LED_GREEN_STATE_PERIOD_MS 2000


typedef struct
{
    PinData *red_led;
    PinData *green_led;
    PinData *blue_led;
    int state;
    int green_state_time_elapsed_ms;
} AutonLED;

// REQUIRES: _red_led, _green_led, and _blue_led are PinData objects
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created AutonLED object
AutonLED *new_auton_led(PinData *_red_led, PinData *_green_led, PinData *_blue_led);

// REQUIRES: auton_led is an AutonLED object and 0 <= state <= 3
// MODIFIES: state
// EFFECTS: Changes the auton_led state
void change_auton_led_state(AutonLED *auton_led, int state);

// REQUIRES: auton_led is an AutonLED object.
// This function is expected to be called every 1 ms
// EFFECTS: Changes the auton_led colors.
// This is specifically so the blinking green lights work properly
void update_auton_led_state(AutonLED *auton_led);
