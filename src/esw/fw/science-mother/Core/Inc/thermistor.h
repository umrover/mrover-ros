#pragma once

#include "adc_sensor.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define THERMISTOR_RESISTOR_OHMS 10000.0f
#define THERMISTOR_R_25 10000.0f
#define THERMISTOR_V_1 3.3f


// TH10K Thermistor
typedef struct {
	float temperature;
	uint8_t adc_channel;
	ADCSensor* adc_sensor;
} Thermistor;

// REQUIRES: _adc_channel is the corresponding ADC channel and
// _adc_sensor is a pointer to an ADCSensor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Thermistor object
Thermistor *new_thermistor(ADCSensor* _adc_sensor, uint8_t _adc_channel);

// REQUIRES: thermistor is a Thermistor object
// MODIFIES: temperature
// EFFECTS: Updates temperature of thermistor
void update_thermistor_temperature(Thermistor* therm);

// REQUIRES: thermistor is a Thermistor object
// MODIFIES: nothing
// EFFECTS: Get temperature of thermistor in degrees Celsius
float get_thermistor_temperature(Thermistor* therm);
