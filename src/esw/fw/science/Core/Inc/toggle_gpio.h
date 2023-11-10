#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32g4xx_hal_gpio.h"

#pragma once

typedef struct 
{
    /* data */
	GPIO_TypeDef* p;
    uint16_t pin;
} Toggle_GPIO;

// REQUIRES: obj is an obj object
// MODIFIES: obj
// EFFECTS: set the value of pin with value
void set_pin_value(Toggle_GPIO* toggle, bool value);

// REQUIRES: obj is an obj object
// MODIFIES: 
// EFFECTS: get the value of pin
bool get_pin_value(Toggle_GPIO *toggle);

// REQUIRES: p and pin are valid GPIO_Init and pin numbers
// MODIFIES: 
// EFFECTS: creates a new pointer to a Toggle_GPIO set to the pin info provided
Toggle_GPIO* new_toggle_gpio(GPIO_TypeDef* p_in, uint16_t pin_in);
