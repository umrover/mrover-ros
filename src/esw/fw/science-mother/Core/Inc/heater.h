#pragma once

#include <stdbool.h>

#include "pin_data.h"
#include "thermistor.h"

#define MAX_HEATER_TEMP 65.0f
#define HEATER_WATCHDOG_TIMEOUT 3000

typedef struct
{
    PinData *heater_pin;
    Thermistor *thermistor;
    bool auto_shutoff;
    bool is_on;
    bool send_auto_shutoff;
    bool send_on;
    uint32_t ms_since_last_received_heater_msg;
} Heater;

// REQUIRES: _heater_pin is a pointer to a PinData object and therm
// is a pointer to a Thermistor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Heater object
Heater *new_heater(PinData *_heater_pin, Thermistor *_thermistor);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Ticks the ms_since_last_received_heater_msg.
// If it's been too long since last received a heater msg AND it's ON, then turn it off.
void tick_heater(Heater *heater);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Updates temperature of heater thermistor
void update_heater_temperature(Heater *heater);

// REQUIRES: nothing
// MODIFIES: is_on
// EFFECTS:  Turns heater off if it is on, thermistor temperature exceeds
// permitted temperature, and auto_shutoff is enabled
void update_heater_state(Heater *heater);

// REQUIRES: state is either false or true, representing off or on
// MODIFIES: is_on
// EFFECTS:  Turn heater off if state is false. Turn heater on if state is true
// AND either temperature is lower than permitted temperature OR auto_shutoff is
// disabled
void change_heater_state(Heater *heater, bool state);
