#pragma once

#include "stm32g4xx_hal.h"

#include "adc_sensor.h"
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DIAG_TEMP_COEFFICIENT 0.0064f // Ohms / Celsius
#define DIAG_TEMP_25_DEGREE_RESISTANCE 47000 // Ohms

// Part Link
// https://www.ti.com/lit/ds/symlink/tmp64.pdf
typedef struct {
    uint8_t channel;
    float temp;
    ADCSensor* adc_sensor;
} DiagTempSensor;

// REQUIRES: _adc_channel is the corresponding ADC channel and
// _adc_sensor is a pointer to an ADCSensor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created temp sensor object
DiagTempSensor* new_diag_temp_sensor(ADCSensor* adc_sensor, int channel);

// REQUIRES: valid temp sensor
// MODIFIES: stored sensor value
// EFFECTS: updates the sensor value
void update_diag_temp_sensor_val(DiagTempSensor* sensor);

// REQUIRES: valid temp sensor
// MODIFIES: nothing
// EFFECTS: returns the stored value for amps
float get_diag_temp_sensor_val(DiagTempSensor* sensor);
