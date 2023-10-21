#include "toggle_gpio.h"

// REQUIRES: led is an led object
// MODIFIES: pin
// EFFECTS: set the value of pin ot value
void set_pin_value(Toggle_GPIO* led, bool value) {
    HAL_GPIO_WritePin(led->p, led->pin, value);
}

// REQUIRES: led is an led object
// MODIFIES: 
// EFFECTS: get the value of pin
bool get_pin_value(Toggle_GPIO *led) {
    return HAL_GPIO_ReadPin(led->p, led->pin);
}