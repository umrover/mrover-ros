#ifndef INC_PIN_H_
#define INC_PIN_H_


#include <stdlib.h>
#include "stm32f1xx_hal.h"

#include "stdbool.h"


typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
} Pin;


Pin *new_pin(GPIO_TypeDef *_port, uint16_t _pin);

bool read_pin_value(Pin* pin);

void write_pin_value(Pin* pin, bool val);

#endif /* INC_PIN_H_ */
