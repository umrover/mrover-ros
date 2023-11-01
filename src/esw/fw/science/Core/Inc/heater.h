#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32g4xx_hal_gpio.h"
#include "diag_temp_sensor.h"
#include "toggle_gpio.h"
#define MAX_HEATER_TEMP 65.0f
#define HEATER_WATCHDOG_TIMEOUT 3000

typedef struct 
{
    /* data */
    Toggle_GPIO* pin;
    bool is_on;
    DiagTempSensor* thermistor;
    bool auto_shutoff;
    uint32_t ms_since_last_received_heater_msg;
}Heater;

// REQUIRES: nothing
// MODIFIES: is_on
// EFFECTS:  Turns heater off if it is on AND thermistor temperature exceeds
// permitted temperature AND auto_shutoff is enabled.
void update_heater_state(Heater* heater);

// REQUIRES: new_shutoff_state is a boolean value, Heater* is a heater pointer
// MODIFIES: auto_shutoff
// EFFECTS: Sets auto_shutoff of the heater to the state inputted
void set_autoshutoff(Heater* heater, bool new_auto_shutoff_state);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Ticks the ms_since_last_received_heater_msg.
// If it's been too long since last received a heater msg AND it's ON, then turn it off.
void tick_heater(Heater *heater);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Updates temperature of heater thermistor
void update_heater_temperature(Heater *heater);