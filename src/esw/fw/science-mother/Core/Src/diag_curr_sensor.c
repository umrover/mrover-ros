////////////////////////////////
//      ACHS-7121 Current Sensor Nucleo Hardware Interface
//      Written by:
//      Jess Wu
//      jessyw@umich.edu
////////////////////////////////

#include "diag_curr_sensor.h"


// REQUIRES: _adc_channel is the corresponding ADC channel and
// _adc_sensor is a pointer to an ADCSensor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created current sensor object
DiagCurrentSensor* new_diag_current_sensor(ADCSensor* adc_sensor, uint8_t channel) {
    DiagCurrentSensor* current_sensor = (DiagCurrentSensor*) malloc(sizeof(DiagCurrentSensor));
    current_sensor->adc_sensor = adc_sensor;
    current_sensor->channel = channel;
    current_sensor->amps = 0;

    return current_sensor;
}

// REQUIRES: valid current sensor
// MODIFIES: stored sensor value
// EFFECTS: updates the sensor value
void update_diag_current_sensor_val(DiagCurrentSensor* sensor) {
    // sensor returns volts (I think) so get to millivolts and solve the proportion for amps then add the offset. (vcc/2)
	float measured_volts = get_adc_sensor_value(sensor, sensor->channel);
    sensor->amps = (1000 * (measured_volts / DIAG_CURR_MV_PER_AMP)) - DIAG_CURR_VCC/2;
}

// REQUIRES: valid current sensor
// MODIFIES: nothing
// EFFECTS: returns the stored value for amps
float get_diag_current_sensor_val(DiagCurrentSensor* sensor) {
    return sensor->amps;
}
