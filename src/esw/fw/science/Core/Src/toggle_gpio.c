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

// REQUIRES: p and pin are valid GPIO_Init and pin numbers
// MODIFIES: 
// EFFECTS: creates a new pointer to a Toggle_GPIO set to the pin info provided
Toggle_GPIO* new_toggle_gpio(GPIO_InitTypeDef p_in, uint16_t pin_in){
    Toggle_GPIO* toggle = malloc(sizeof(Toggle_GPIO));
    toggle->p = p_in;
    toggle->pin = pin_in;
    return toggle;
}