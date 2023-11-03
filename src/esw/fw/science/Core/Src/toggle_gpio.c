#include "toggle_gpio.h"

// REQUIRES: toggle is an Toggle_GPIO object
// MODIFIES: pin
// EFFECTS: set the value of pin ot value
void set_pin_value(Toggle_GPIO* toggle, bool value) {
    HAL_GPIO_WritePin(toggle->p, toggle->pin, value);
}

// REQUIRES: toggle is a toggle_gpio object
// MODIFIES: 
// EFFECTS: get the value of pin
bool get_pin_value(Toggle_GPIO *toggle) {
    return HAL_GPIO_ReadPin(toggle->p, toggle->pin);
}


