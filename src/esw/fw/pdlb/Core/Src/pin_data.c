#include "pin_data.h"

/** PUBLIC FUNCTIONS **/

// REQUIRES: _port and _pin corresponds to
// the port and pin and _is_output is boolean
// that is true if the pin is an output pin.
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created PinData object
PinData *new_pin_data(GPIO_TypeDef *_port, uint16_t _pin, bool _is_output) {
    PinData *pin_data = (PinData*) malloc(sizeof(PinData));
	pin_data->port = _port;
    pin_data->pin = _pin;
    pin_data->is_output = _is_output;
	return pin_data;
}

// REQUIRES: pin_data is PinData and value is 0 or 1
// MODIFIES: nothing
// EFFECTS: Sets pin to value
void set_pin_value(PinData *pin_data, bool value) {
	if (!pin_data->is_output) {
		return;
	}
	HAL_GPIO_WritePin(pin_data->port, pin_data->pin, value == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

// REQUIRES: pin_data is PinData
// MODIFIES: nothing
// EFFECTS: Returns value of pin
bool get_pin_value(PinData *pin_data) {
	if (pin_data->is_output) {
		return false;
	}
	bool value = HAL_GPIO_ReadPin(pin_data->port, pin_data->pin);
	return value;
}
/** PRIVATE FUNCTIONS MAY BE IN SOURCE FILE ONLY **/
