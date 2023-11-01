#pragma once

#include "main.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>

//#include "heater.h"
//#include "servo.h"

// The communication bridge between the Jetson and the chip
typedef struct
{
	UART_HandleTypeDef *uart;
	char uart_buffer[30];
} Bridge;

// REQUIRES: uart is the uart channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Bridge object
Bridge *new_bridge(UART_HandleTypeDef *_uart);

//// REQUIRES: bridge, heater, mosfet_device, and servo are objects
//// MODIFIES: Nothing
//// EFFECTS: Receives the message and processes it
//void receive_bridge(Bridge *bridge, Heater *heaters[3], PinData *mosfet_pins[12], Servo *servos[3]);

//// REQUIRES: bridge and mosfet_device are objects
//// MODIFIES: Nothing
//// EFFECTS: Receives the message if it is a mosfet message in the format:
//// "$MOSFET,<DEVICE>,<ENABLE>"
//void receive_bridge_mosfet_cmd(Bridge *bridge, PinData *mosfet_pins[12]);
//
//// REQUIRES: bridge and servos are objects
//// MODIFIES: Nothing
//// EFFECTS: Receives the message if it is a servo message in the format:
//// "$SERVO,<SERVO ID>,<ANGLE>"
//void receive_bridge_servo_cmd(Bridge *bridge, Servo *servos[3]);
//
//// REQUIRES: bridge and heaters are objects
//// MODIFIES: Nothing
//// EFFECTS: Receives the message if it is an auto shutoff message in the format:
//// "$AUTO_SHUTOFF,<VAL>"
//void receive_bridge_auto_shutoff_cmd(Bridge *bridge, Heater *heaters[3]);
//
//// REQUIRES: bridge and heaters are objects
//// MODIFIES: Nothing
//// EFFECTS: Receives the message if it is a heater message in the format:
//// "$HEATER_CMD,<DEVICE>,<ENABLE>"
//void receive_bridge_heater_cmd(Bridge *bridge, Heater *heaters[3]);
//
//// REQUIRES: nothing
//// MODIFIES: nothing
//// EFFECTS: Sends diagnostic current and thermistor data in format:
//// $DIAG,,<TEMP_0>,<TEMP_1>,<TEMP_2>,<CURR_0>,<CURR_1>,<CURR_2>
//void bridge_send_diagnostic(Bridge *bridge, float temps[3], float currs[3]);

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends spectral data in format:
// "$SPECTRAL, <ch0>, <ch1>, <ch2>, <ch3>, <ch4>, <ch5>""
void bridge_send_spectral(Bridge *bridge, uint16_t ch_data[6]);

//// REQUIRES: nothing
//// MODIFIES: nothing
//// EFFECTS: Sends science temperatures in format:
//// "$SCIENCE_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>"
//void bridge_send_science_thermistors(Bridge *bridge, float temps[3]);
//
//// REQUIRES: nothing
//// MODIFIES: nothing
//// EFFECTS: Sends heater auto shutoff in format:
//// "$AUTO_SHUTOFF,<VAL>"
//void bridge_send_heater_auto_shutoff(Bridge *bridge, bool state);
//
//// REQUIRES: nothing
//// MODIFIES: nothing
//// EFFECTS: Sends heater state in format:
//// "$HEATER_DATA,<STATE_0>,<STATE_1>,<STATE_2>"
//void bridge_send_heater_state(Bridge *bridge, bool states[3]);
