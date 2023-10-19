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
}uv_led;

// REQUIRES: led is an white_led object
// MODIFIES: pin
// EFFECTS: set the value of pin ot value
void set_pin_value(uv_led *led, bool value);

// REQUIRES: led is an white_led object
// MODIFIES: 
// EFFECTS: get the value of pin
bool get_pin_value(uv_led *led);
