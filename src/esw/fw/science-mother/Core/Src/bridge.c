#include "bridge.h"

// REQUIRES: uart is the uart channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Bridge object
Bridge *new_bridge(UART_HandleTypeDef *_uart) {
    Bridge *bridge = (Bridge *)malloc(sizeof(Bridge));
    bridge->uart = _uart;
	bridge->ms_since_last_received_uart_msg = 0;
	bridge->UART_watchdog_flag = false;
	for (int i = 0; i < UART_BUFFER_SIZE; ++i) {
		bridge->uart_buffer[i] = 0;
	}
	for (int i = 0; i < TOTAL_UART_MESSAGE_SIZE; ++i) {
		bridge->message_buffer[i] = 0;
	}
	bridge->msg_length_counter = 0;

    return bridge;
}

void UART_CH_reset(Bridge *UART_channel) {
	HAL_UART_DeInit(UART_channel->uart);
	HAL_UART_Init(UART_channel->uart);
	UART_channel->UART_watchdog_flag = false;
	HAL_UART_Receive_DMA(UART_channel->uart, (uint8_t *)UART_channel->uart_buffer, sizeof(UART_channel->uart_buffer));
}

void UART_CH_tick(Bridge *UART_channel) {
    UART_channel->ms_since_last_received_uart_msg += 1;
    if (UART_channel->ms_since_last_received_uart_msg >= UART_WATCHDOG_TIMEOUT) {
        UART_channel->ms_since_last_received_uart_msg = 0;
        UART_channel->UART_watchdog_flag = true;
    }
}
// REQUIRES: bridge, heater, mosfet_device, servo, and auton_led are objects
// MODIFIES: Nothing
// EFFECTS: Receives the message and processes it
void receive_bridge(Bridge *bridge, Heater *heaters[3], PinData *mosfet_pins[12], Servo *servos[3], AutonLED *auton_led) {
	fill_message_buffer(bridge);
	//If the message has been created

	if(bridge->msg_length_counter >= 30)
	{
		// Could be redundant check for $, but I'll keep it for now
		// Also, before the first message is received, it will come in here
		// because msg_length_counter starts at 30.
		// But it should then get kicked out immediately, so prob no issues
		if (bridge->message_buffer[0] == '$') {
			// Expect it always to be a $ sign.
			if (bridge->message_buffer[1] == 'M') {
				receive_bridge_mosfet_cmd(bridge, mosfet_pins);
			}
			else if (bridge->message_buffer[1] == 'S') {
				receive_bridge_servo_cmd(bridge, servos);
			}
			else if (bridge->message_buffer[1] == 'A') {
				receive_bridge_auto_shutoff_cmd(bridge, heaters);
			}
			else if (bridge->message_buffer[1] == 'H') {
				receive_bridge_heater_cmd(bridge, heaters);
			}
			else if (bridge->message_buffer[1] == 'L') {
				receive_bridge_auton_led_cmd(bridge, auton_led);
			}
			else if(bridge->message_buffer[1] == 'W'){
				// This is looking for the watchdog NMEA message
				bridge->ms_since_last_received_uart_msg = 0;
				bridge->UART_watchdog_flag = false;
			}
		}
	}
	HAL_UART_Receive_DMA(bridge->uart, (uint8_t *)bridge->uart_buffer, sizeof(bridge->uart_buffer));
}

// REQUIRES: bridge and mosfet_device are objects
// MODIFIES: Nothing
// EFFECTS: Receives the message if it is a mosfet message in the format:
// "$MOSFET,<DEVICE>,<ENABLE>"
void receive_bridge_mosfet_cmd(Bridge *bridge , PinData *mosfet_pins[12]) {
	if (bridge->message_buffer[0] != '$' || bridge->message_buffer[1] != 'M') {
		// This should be asserted.
		// The function should not have been called if it was not the correct message
		return;
	}

	char *identifier = strtok(bridge->message_buffer, ",");

	if (!strcmp(identifier,"$MOSFET")){
		int device = -1;
		bool state = false;

		device = atoi(strtok(NULL, ","));
		state = atoi(strtok(NULL, ","));

		if (0 <= device && device < 12) {
			set_pin_value(mosfet_pins[device], state);
		}
	}
}

// REQUIRES: bridge and servos are objects
// MODIFIES: Nothing
// EFFECTS: Receives the message if it is a servo message in the format:
// "$SERVO,<SERVO ID>,<ANGLE>"
void receive_bridge_servo_cmd(Bridge *bridge, Servo *servos[3]) {
	if (bridge->message_buffer[0] != '$' || bridge->message_buffer[1] != 'S') {
		// This should be asserted.
		// The function should not have been called if it was not the correct message
		return;
	}

	char *identifier = strtok(bridge->message_buffer, ",");

	if (!strcmp(identifier,"$SERVO")){
		int device = -1;
		int angle = 0;

		device = atoi(strtok(NULL, ","));
		angle = atoi(strtok(NULL, ","));

		if (0 <= device && device < 3) {
			set_servo_angle(servos[device], angle);
		}
	}
}

// REQUIRES: bridge and heaters are objects
// MODIFIES: Nothing
// EFFECTS: Receives the message if it is an auto shutoff message in the format:
// "$AUTO_SHUTOFF,<VAL>"
void receive_bridge_auto_shutoff_cmd(Bridge *bridge, Heater *heaters[3]) {
	if (bridge->message_buffer[0] != '$' || bridge->message_buffer[1] != 'A') {
		// This should be asserted.
		// The function should not have been called if it was not the correct message
		return;
	}

	char *identifier = strtok(bridge->message_buffer, ",");

	if (!strcmp(identifier,"$AUTO_SHUTOFF")){
		bool state = false;

		state = atoi(strtok(NULL, ","));

		for (size_t i = 0; i < 3; ++i) {
			heaters[i]->auto_shutoff = state;
			heaters[i]->send_auto_shutoff = true;
		}

	}
}

// REQUIRES: bridge and heaters are objects
// MODIFIES: Nothing
// EFFECTS: Receives the message if it is a heater message in the format:
// "$HEATER_CMD,<DEVICE>,<ENABLE>"
void receive_bridge_heater_cmd(Bridge *bridge, Heater *heaters[3]) {
	if (bridge->message_buffer[0] != '$' || bridge->message_buffer[1] != 'H') {
		// This should be asserted.
		// The function should not have been called if it was not the correct message
		return;
	}

	char *identifier = strtok(bridge->message_buffer, ",");

	if (!strcmp(identifier,"$HEATER_CMD")){
		int device = -1;
		bool state = false;

		device = atoi(strtok(NULL, ","));
		state = atoi(strtok(NULL, ","));

		if (0 <= device && device < 3) {
			heaters[device]->ms_since_last_received_heater_msg = 0;
			change_heater_state(heaters[device], state);
			heaters[device]->send_on = true;
		}
	}
}

// REQUIRES: bridge and auton_led are objects
// MODIFIES: Nothing
// EFFECTS: Receives the message if it is a auton_led message in the format:
// "$LED, <REQUESTED_STATE>"
// where number is 0 for red, 1 for blinking green, 2 for blue, and 3 for off
void receive_bridge_auton_led_cmd(Bridge *bridge, AutonLED *auton_led) {
	if (bridge->message_buffer[0] != '$' || bridge->message_buffer[1] != 'L') {
		// This should be asserted.
		// The function should not have been called if it was not the correct message
		return;
	}

	char *identifier = strtok(bridge->message_buffer, ",");

	if (!strcmp(identifier,"$LED")){
		int requested_state = -1;

		requested_state = atoi(strtok(NULL, ","));

		if (0 <= requested_state && requested_state < 4) {
			change_auton_led_state(auton_led, requested_state);
		}
	}
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends diagnostic current and thermistor data in format:
// $DIAG,,<TEMP_0>,<TEMP_1>,<TEMP_2>,<CURR_0>,<CURR_1>,<CURR_2>
void bridge_send_diagnostic(Bridge *bridge, float temps[3], float currs[3]) {
    char msg[90];

    snprintf(msg, sizeof(msg), "$DIAG,%f,%f,%f,%f,%f,%f,", temps[0], temps[1],
    		temps[2], currs[0], currs[1], currs[2]);

    HAL_Delay(100);
    HAL_UART_Transmit(bridge->uart, (uint8_t *)msg, 90, 200);
    HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends spectral data in format:
// "$SPECTRAL, <ch0>, <ch1>, <ch2>, <ch3>, <ch4>, <ch5>""
void bridge_send_spectral(Bridge *bridge, uint16_t ch_data[6]) {
    char msg[50];

    snprintf(msg, sizeof(msg), "$SPECTRAL,%u,%u,%u,%u,%u,%u,", ch_data[0],
    		ch_data[1], ch_data[2], ch_data[3], ch_data[4],
			ch_data[5]);

    HAL_Delay(100);
    HAL_UART_Transmit(bridge->uart, (uint8_t *)msg, 50, 200);
    HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends science temperatures in format:
// "$SCIENCE_TEMP,<TEMP_0>,<TEMP_1>,<TEMP_2>"
void bridge_send_science_thermistors(Bridge *bridge, float temps[3]) {
    char msg[50];
    snprintf(msg, sizeof(msg), "$SCIENCE_TEMP,%f,%f,%f,", temps[0], temps[1], temps[2]);
    HAL_Delay(100);
    HAL_UART_Transmit(bridge->uart, (uint8_t *)msg, 50, 200);
    HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends heater auto shutoff in format:
// "$AUTO_SHUTOFF,<VAL>"
void bridge_send_heater_auto_shutoff(Bridge *bridge, bool state) {
	char msg[20];
	snprintf(msg, sizeof(msg), "$AUTO_SHUTOFF,%i,", state);
	HAL_Delay(100);
	HAL_UART_Transmit(bridge->uart, (uint8_t *)msg, 20, 200);
	HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: nothing
// EFFECTS: Sends heater state in format:
// "$HEATER_DATA,<STATE_0>,<STATE_1>,<STATE_2>"
void bridge_send_heater_state(Bridge *bridge, bool states[3]) {
	char msg[25];
	snprintf(msg, sizeof(msg), "$HEATER_DATA,%i,%i,%i,", states[0], states[1], states[2]);
	HAL_Delay(100);
	HAL_UART_Transmit(bridge->uart, (uint8_t *)msg, 25, 200);
	HAL_Delay(100);
}

// REQUIRES: nothing
// MODIFIES: bridge->message_buffer
// EFFECTS: Starts creating the message if the uart buffer reads in a $
//          Fills in the 30 char message buffer
//          example finished message: "$SERVO..."
void fill_message_buffer(Bridge *bridge) {
	// Look for $ in uart buffer
	// Sets msg_length_counter to 0 to prepare to create message
	if(bridge->uart_buffer[0] =='$')
	{
		bridge->msg_length_counter = 0;
	}
	// create the message if msg_length_counter is a valid index
	int bridge_msg_length_counter = bridge->msg_length_counter;
	if (bridge_msg_length_counter < 30)
	{	
		bridge->message_buffer[bridge_msg_length_counter] = bridge->uart_buffer[0];
		bridge->msg_length_counter++;
	}
}
