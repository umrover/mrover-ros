#include "bridge.h"

// REQUIRES: uart is the uart channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Bridge object
Bridge *new_bridge(UART_HandleTypeDef *_uart)
{
    Bridge *bridge = (Bridge *)malloc(sizeof(Bridge));
    bridge->uart = _uart;
	for (int i = 0; i < 30; ++i) {
		bridge->uart_buffer[i] = 0;
	}
	HAL_UART_Receive_DMA(bridge->uart, (uint8_t *)bridge->uart_buffer, sizeof(bridge->uart_buffer));

    return bridge;
}

//// REQUIRES: bridge, heater, mosfet_device, and servo are objects
//// MODIFIES: Nothing
//// EFFECTS: Receives the message and processes it
//void receive_bridge(Bridge *bridge, Heater *heaters[3], PinData *mosfet_pins[12], Servo *servos[3]) {
//	HAL_UART_Receive_DMA(bridge->uart, (uint8_t *)bridge->uart_buffer, sizeof(bridge->uart_buffer));
//	if (bridge->uart_buffer[0] == '$') {
//		// Expect it always to be a $ sign.
//		if (bridge->uart_buffer[1] == 'M') {
//			receive_bridge_mosfet_cmd(bridge, mosfet_pins);
//		}
//		else if (bridge->uart_buffer[1] == 'S') {
//			receive_bridge_servo_cmd(bridge, servos);
//		}
//		else if (bridge->uart_buffer[1] == 'A') {
//			receive_bridge_auto_shutoff_cmd(bridge, heaters);
//		}
//		else if (bridge->uart_buffer[1] == 'H') {
//			receive_bridge_heater_cmd(bridge, heaters);
//		}
//	}
//}
//
//// REQUIRES: bridge and mosfet_device are objects
//// MODIFIES: Nothing
//// EFFECTS: Receives the message if it is a mosfet message in the format:
//// "$MOSFET,<DEVICE>,<ENABLE>"
//void receive_bridge_mosfet_cmd(Bridge *bridge , PinData *mosfet_pins[12]) {
//	if (bridge->uart_buffer[0] != '$' || bridge->uart_buffer[1] != 'M') {
//		// This should be asserted.
//		// The function should not have been called if it was not the correct message
//		return;
//	}
//
//	char *identifier = strtok(bridge->uart_buffer, ",");
//
//	if (!strcmp(identifier,"$MOSFET")){
//		int device = -1;
//		bool state = false;
//
//		device = atoi(strtok(NULL, ","));
//		state = atoi(strtok(NULL, ","));
//
//		if (0 <= device && device < 12) {
//			set_pin_value(mosfet_pins[device], state);
//		}
//	}
//}
//
//// REQUIRES: bridge and servos are objects
//// MODIFIES: Nothing
//// EFFECTS: Receives the message if it is a servo message in the format:
//// "$SERVO,<SERVO ID>,<ANGLE>"
//void receive_bridge_servo_cmd(Bridge *bridge, Servo *servos[3]) {
//	if (bridge->uart_buffer[0] != '$' || bridge->uart_buffer[1] != 'S') {
//		// This should be asserted.
//		// The function should not have been called if it was not the correct message
//		return;
//	}
//
//	char *identifier = strtok(bridge->uart_buffer, ",");
//
//	if (!strcmp(identifier,"$SERVO")){
//		int device = -1;
//		int angle = 0;
//
//		device = atoi(strtok(NULL, ","));
//		angle = atoi(strtok(NULL, ","));
//
//		if (0 <= device && device < 3) {
//			set_servo_angle(servos[device], angle);
//		}
//	}
//}
//
//// REQUIRES: bridge and heaters are objects
//// MODIFIES: Nothing
//// EFFECTS: Receives the message if it is an auto shutoff message in the format:
//// "$AUTO_SHUTOFF,<VAL>"
//void receive_bridge_auto_shutoff_cmd(Bridge *bridge, Heater *heaters[3]) {
//	if (bridge->uart_buffer[0] != '$' || bridge->uart_buffer[1] != 'A') {
//		// This should be asserted.
//		// The function should not have been called if it was not the correct message
//		return;
//	}
//
//	char *identifier = strtok(bridge->uart_buffer, ",");
//
//	if (!strcmp(identifier,"$AUTO_SHUTOFF")){
//		bool state = false;
//
//		state = atoi(strtok(NULL, ","));
//
//		for (size_t i = 0; i < 3; ++i) {
//			heaters[i]->auto_shutoff = state;
//			heaters[i]->send_auto_shutoff = true;
//		}
//
//	}
//}
//
//// REQUIRES: bridge and heaters are objects
//// MODIFIES: Nothing
//// EFFECTS: Receives the message if it is a heater message in the format:
//// "$HEATER_CMD,<DEVICE>,<ENABLE>"
//void receive_bridge_heater_cmd(Bridge *bridge, Heater *heaters[3]) {
//	if (bridge->uart_buffer[0] != '$' || bridge->uart_buffer[1] != 'H') {
//		// This should be asserted.
//		// The function should not have been called if it was not the correct message
//		return;
//	}
//
//	char *identifier = strtok(bridge->uart_buffer, ",");
//
//	if (!strcmp(identifier,"$HEATER_CMD")){
//		int device = -1;
//		bool state = false;
//
//		device = atoi(strtok(NULL, ","));
//		state = atoi(strtok(NULL, ","));
//
//		if (0 <= device && device < 3) {
//			change_heater_state(heaters[device], state);
//			heaters[device]->send_on = true;
//		}
//	}
//}
//
//// REQUIRES: nothing
//// MODIFIES: nothing
//// EFFECTS: Sends diagnostic current and thermistor data in format:
//// $DIAG,,<TEMP_0>,<TEMP_1>,<TEMP_2>,<CURR_0>,<CURR_1>,<CURR_2>
//void bridge_send_diagnostic(Bridge *bridge, float temps[3], float currs[3])
//{
//    char msg[150];
//
//    snprintf(msg, sizeof(msg), "$DIAG,%f,%f,%f,%f,%f,%f,", temps[0], temps[1],
//    		temps[2], currs[0], currs[1], currs[2]);
//
//    HAL_Delay(100);
//    HAL_UART_Transmit(bridge->uart, (uint8_t *)msg, 150, 200);
//    HAL_Delay(100);
//}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends spectral data in format:
// "$SPECTRAL, <ch0>, <ch1>, <ch2>, <ch3>, <ch4>, <ch5>""
void bridge_send_spectral(Bridge *bridge, uint16_t ch_data[6])
{
    char msg[150];

    snprintf(msg, sizeof(msg), "$SPECTRAL,%u,%u,%u,%u,%u,%u,", ch_data[0],
    		ch_data[1], ch_data[2], ch_data[3], ch_data[4],
			ch_data[5]);

    HAL_Delay(100);
    HAL_UART_Transmit(bridge->uart, (uint8_t *)msg, 150, 200);
    HAL_Delay(100);
}

//// REQUIRES: nothing
//// MODIFIES: nothing
//// EFFECTS: Sends science temperatures in format:
//// "$SCIENCE_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>"
//void bridge_send_science_thermistors(Bridge *bridge, float temps[3])
//{
//    char msg[150];
//    snprintf(msg, sizeof(msg), "$SCIENCE_TEMP,%f,%f,%f,", temps[0], temps[1], temps[2]);
//    HAL_Delay(100);
//    HAL_UART_Transmit(bridge->uart, (uint8_t *)msg, 150, 200);
//    HAL_Delay(100);
//}
//
//// REQUIRES: nothing
//// MODIFIES: nothing
//// EFFECTS: Sends heater auto shutoff in format:
//// "$AUTO_SHUTOFF,<VAL>"
//void bridge_send_heater_auto_shutoff(Bridge *bridge, bool state) {
//	char msg[150];
//	snprintf(msg, sizeof(msg), "$AUTO_SHUTOFF,%i,", state);
//	HAL_Delay(100);
//	HAL_UART_Transmit(bridge->uart, (uint8_t *)msg, 150, 200);
//	HAL_Delay(100);
//}
//
//// REQUIRES: nothing
//// MODIFIES: nothing
//// EFFECTS: Sends heater state in format:
//// "$HEATER_DATA,<STATE_0>,<STATE_1>,<STATE_2>"
//void bridge_send_heater_state(Bridge *bridge, bool states[3]) {
//	char msg[150];
//	snprintf(msg, sizeof(msg), "$HEATER_DATA,%i,%i,%i,", states[0], states[1], states[2]);
//	HAL_Delay(100);
//	HAL_UART_Transmit(bridge->uart, (uint8_t *)msg, 150, 200);
//	HAL_Delay(100);
//}
