#include "heater.h"

// REQUIRES: nothing
// MODIFIES: is_on
// EFFECTS:  Turns heater off
// The difference between this and
void turn_heater_off(Heater *heater)
{
	set_pin_value(heater->heater_pin, false);
    heater->is_on = false;
}

// REQUIRES: nothing
// MODIFIES: is_on
// EFFECTS:  Turns heater on
void turn_heater_on(Heater *heater)
{
	set_pin_value(heater->heater_pin, true);
    heater->is_on = true;
}

// REQUIRES: _heater_pin is a pointer to a PinData object and therm
// is a pointer to a Thermistor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Heater object
Heater *new_heater(PinData *_heater_pin, Thermistor *_thermistor)
{
    Heater *heater = (Heater *)malloc(sizeof(Heater));

    heater->heater_pin = _heater_pin;
    heater->thermistor = _thermistor;
    heater->auto_shutoff = true;
    heater->is_on = false;
    heater->send_auto_shutoff = true;
    heater->send_on = true;
    heater->ms_since_last_received_heater_msg = 0;
    return heater;
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Ticks the ms_since_last_received_heater_msg.
// If it's been too long since last received a heater msg AND it's ON, then turn it off.
void tick_heater(Heater *heater) {
	heater->ms_since_last_received_heater_msg += 1;
	if (heater->ms_since_last_received_heater_msg >= HEATER_WATCHDOG_TIMEOUT) {
		change_heater_state(heater, false);
		heater->ms_since_last_received_heater_msg = 0;
	}
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Updates temperature of heater thermistor
void update_heater_temperature(Heater *heater)
{
    update_thermistor_temperature(heater->thermistor);
}

// REQUIRES: nothing
// MODIFIES: is_on
// EFFECTS:  Turns heater off if it is on AND thermistor temperature exceeds
// permitted temperature AND auto_shutoff is enabled.
void update_heater_state(Heater *heater)
{
    if (heater->is_on && get_thermistor_temperature(heater->thermistor) >= MAX_HEATER_TEMP && heater->auto_shutoff)
    {
    	heater->send_on = true;
        turn_heater_off(heater);
    }
}

// REQUIRES: state is either false or true, representing off or on
// MODIFIES: is_on
// EFFECTS:  Turn heater off if state is false. Turn heater on if state is true
// AND either temperature is lower than permitted temperature OR auto_shutoff is
// disabled
void change_heater_state(Heater *heater, bool state)
{
    if (!state)
    {
    	if (heater->is_on) {
    		heater->send_on = true;
    		turn_heater_off(heater);
    	}
    }
    else if (state && (get_thermistor_temperature(heater->thermistor) < MAX_HEATER_TEMP || !heater->auto_shutoff))
    {
    	if (!heater->is_on) {
    		heater->send_on = true;
			turn_heater_on(heater);
    	}
    }
}

// REQUIRES: state is either false or true, representing off or on
// MODIFIES: auto_shutoff
// EFFECTS:  Turn auto_shutoff on if state is true OR off if state is false
void change_heater_auto_shutoff(Heater *heater, bool state)
{
    if (!state)
    {
        heater->auto_shutoff = true;
    }
    else
    {
        heater->auto_shutoff = false;
    }
}
