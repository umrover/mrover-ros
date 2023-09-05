#include "pin.h"

Pin *new_pin(GPIO_TypeDef *_port, uint16_t _pin) {
    Pin *pin = (Pin *) malloc(sizeof(Pin));
    pin->port = _port;
    pin->pin = _pin;

    return pin;
}

bool read_pin_value(Pin* pin) {
	return HAL_GPIO_ReadPin(pin->port, pin->pin);
}

void write_pin_value(Pin* pin, bool val) {
	HAL_GPIO_WritePin(pin->port, pin->pin, val);
}
