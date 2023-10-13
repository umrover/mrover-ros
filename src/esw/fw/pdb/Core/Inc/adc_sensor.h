#pragma once

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdlib.h>


// ADCSensor: A struct that houses multiple ADC sensors, accessible through channel number
// adc: The MCU's ADC1 or ADC2
// total_channels: Total number of sensors connected to this struct
// values: An array that holds the raw values coming from the sensors
typedef struct {
	ADC_HandleTypeDef *adc;
	uint8_t total_channels;
	uint16_t values[10];
} ADCSensor;

// REQUIRES: hadc is the adc and _total_channels are the total channels.
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created ADCSensor object
ADCSensor *new_adc_sensor(ADC_HandleTypeDef *hadc, uint8_t _total_channels);

// REQUIRES: adc_sensor is an ADCSensor object and channel is the index
// MODIFIES: nothing
// EFFECTS: Returns the currently stored value of trigger.
// Expect an integer between 0 and 4096.
uint16_t get_adc_sensor_value(ADCSensor *adc_sensor, uint8_t channel);

// REQUIRES: adc_sensor is an ADCSensor object
// MODIFIES: values
// EFFECTS: Updates the stored value of value.
void update_adc_sensor_values(ADCSensor *adc_sensor);
