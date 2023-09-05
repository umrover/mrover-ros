#pragma once

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#define PIN_IS_OUTPUT true
#define PIN_IS_INPUT false

// Used to store the data of a pin
typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
	bool is_output;
} PinData;

/** PUBLIC FUNCTIONS **/

// REQUIRES: _port and _pin corresponds to
// the port and pin and _is_output is boolean
// that is true if the pin is an output pin.
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created PinData object
PinData *new_pin_data(GPIO_TypeDef *_port, uint16_t _pin, bool _is_output);

// REQUIRES: pin_data is PinData and value is 0 or 1
// MODIFIES: nothing
// EFFECTS: Sets pin to value
void set_pin_value(PinData *pin_data, bool value);

// REQUIRES: pin_data is PinData
// MODIFIES: nothing
// EFFECTS: Returns value of pin
bool get_pin_value(PinData *pin_data);
