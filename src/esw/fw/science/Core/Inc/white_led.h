#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32g4xx_hal_gpio.h"

typedef struct 
{
    /* data */
    GPIO_TypeDef p; 
    uint16_t pin;
}white_led;

void set_pin_value(white_led *led, bool value);
void get_pin_value(white_led *led, bool value);
