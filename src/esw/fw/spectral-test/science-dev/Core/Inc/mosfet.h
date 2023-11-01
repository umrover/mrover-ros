#ifndef MOSFET_H_
#define MOSFET_H_

// A device that is controlled by a MOSFET
typedef struct {
	GPIO_TypeDef *port;
	uint8_t pin;
} MosfetDevice;

// REQUIRES: port is the port and pin is the pin
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Mosfet Device object
MosfetDevice *new_mosfet_device(GPIO_TypeDef *_port, uint8_t _pin);

// REQUIRES: mosfet_device is a Mosfet Device object
// MODIFIES: nothing
// EFFECTS: Turns Mosfet Device on by setting pin high
void turn_mosfet_device_on(MosfetDevice *mosfet_device);

// REQUIRES: mosfet_device is a Mosfet Device object
// MODIFIES: nothing
// EFFECTS: Turns Mosfet Device off by setting pin low
void turn_mosfet_device_off(MosfetDevice *mosfet_device);

// REQUIRES: mosfet_device is a Mosfet Device object and state is either 0 or 1
// MODIFIES: nothing
// EFFECTS: Sets Mosfet Device to desired state
void set_mosfet_device_state(MosfetDevice *mosfet_device, uint8_t state);

#endif

//#endif
