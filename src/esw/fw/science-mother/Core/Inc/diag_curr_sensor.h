#pragma once

#include "stm32f1xx_hal.h"

#include "adc_sensor.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DIAG_CURR_VCC 3.3f
#define DIAG_CURR_MV_PER_AMP 122.1f  // MV_PER_AMP = 185mV/A * VCC/5.0

// ACHS-7121 Hall Effect-based isolated linear current sensor
// No clue whether or not it's a adc sensor or not.
typedef struct {
	uint8_t channel;
    float amps;
    ADCSensor* adc_sensor;
} DiagCurrentSensor;

// REQUIRES: _adc_channel is the corresponding ADC channel and
// _adc_sensor is a pointer to an ADCSensor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created current sensor object
DiagCurrentSensor* new_diag_current_sensor(ADCSensor* adc_sensor, uint8_t channel);

// REQUIRES: valid current sensor
// MODIFIES: stored sensor value
// EFFECTS: updates the sensor value
void update_diag_current_sensor_val(DiagCurrentSensor* sensor);


// REQUIRES: valid current sensor
// MODIFIES: nothing
// EFFECTS: returns the stored value for amps
float get_diag_current_sensor_val(DiagCurrentSensor* sensor);
