#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32g4xx_hal_gpio.h"

typedef struct 
{
    /* data */
    GPIO_InitTypeDef p; 
    uint16_t pin;
}Toggle_GPIO;

// REQUIRES: led is an led object
// MODIFIES: pin
// EFFECTS: set the value of pin ot value
void set_pin_value(Toggle_GPIO* obj, bool value);

// REQUIRES: led is an led object
// MODIFIES: 
// EFFECTS: get the value of pin
bool get_pin_value(Toggle_GPIO* obj);
