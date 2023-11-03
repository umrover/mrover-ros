#include "heater.h"

// REQUIRES: nothing
// MODIFIES: is_on
// EFFECTS:  Turns heater off
void turn_heater_off(Heater *heater)
{
	set_pin_value(heater->pin, false);
    heater->is_on = false;
}

// REQUIRES: nothing
// MODIFIES: is_on
// EFFECTS:  Turns heater on
void turn_heater_on(Heater* heater){
    set_pin_value(heater->pin, true);
    heater->is_on = true;
}

// REQUIRES: nothing
// MODIFIES: is_on
// EFFECTS:  Turns heater off if it is on AND thermistor temperature exceeds
// permitted temperature AND auto_shutoff is enabled.
void update_heater_state(Heater* heater)
{
    if (heater->is_on && get_diag_temp_sensor_val(heater->thermistor) >= MAX_HEATER_TEMP && heater->auto_shutoff)
    {
        turn_heater_off(heater);
    }
}

// REQUIRES: new_shutoff_state is a boolean value, Heater* is a heater pointer
// MODIFIES: auto_shutoff
// EFFECTS: Sets auto_shutoff of the heater to the state inputted
void set_autoshutoff(Heater* heater, bool new_auto_shutoff_state){
    heater->auto_shutoff = new_auto_shutoff_state;
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Ticks the ms_since_last_received_heater_msg.
// If it's been too long since last received a heater msg AND it's ON, then turn it off.
void tick_heater(Heater *heater) {
	heater->ms_since_last_received_heater_msg += 1;
	if (heater->ms_since_last_received_heater_msg >= HEATER_WATCHDOG_TIMEOUT) {
        if (heater->is_on){
            turn_heater_off(heater);
        }
		heater->ms_since_last_received_heater_msg = 0;
	}
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Updates temperature of heater thermistor
void update_heater_temperature(Heater *heater)
{
    update_diag_temp_sensor_val(heater->thermistor);
}

// REQUIRES: parameters are all valid
// MODIFIES: 
// EFFECTS: creates a heater object and returns a pointer to it
Heater* new_heater(Toggle_GPIO* pin, DiagTempSensor* thermistor){
    Heater* heater = malloc(sizeof(Heater));
    heater->is_on = false;
    heater->pin = pin;
    heater->thermistor = thermistor;
    heater->auto_shutoff = false;
    heater->ms_since_last_received_heater_msg = 0;
}