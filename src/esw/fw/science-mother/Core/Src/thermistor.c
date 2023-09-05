////////////////////////////////
//      TH10K Thermistor Nucleo Hardware Interface
//      Written by:
//      Jess Wu
//      jessyw@umich.edu
////////////////////////////////


#include "thermistor.h"

static const float constant_array[4][4] = {
			{3.3570420E-03, 2.5214848E-04, 3.3743283E-06, -6.4957311E-08},
			{3.3540170E-03, 2.5617244E-04, 2.1400943E-06, -7.2405219E-08},
			{3.3530481E-03, 2.5420230E-04, 1.1431163E-06, -6.9383563E-08},
			{3.3536166E-03, 2.5377200E-04, 8.5433271E-07, -8.7912262E-08}
};

// REQUIRES: _adc_channel is the corresponding ADC channel and
// _adc_sensor is a pointer to an ADCSensor object
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Thermistor object
Thermistor *new_thermistor(ADCSensor* _adc_sensor, uint8_t _adc_channel) {
    Thermistor* therms = (Thermistor*) malloc(sizeof(Thermistor));
    therms->temperature = 100;
    therms->adc_channel = _adc_channel;
    therms->adc_sensor = _adc_sensor;

    return therms;
}

// REQUIRES: thermistor is a Thermistor object
// MODIFIES: temperature
// EFFECTS: Updates temperature of thermistor
void update_thermistor_temperature(Thermistor* therm) {
    uint16_t raw_data = get_adc_sensor_value(therm->adc_sensor, therm->adc_channel);

	// done to avoid sending infinity/nan
    raw_data = raw_data > 4094 ? 4094 : raw_data;

    // Logic to get actual Voltage from 12 bit string
    // NOTE pretty sure it is 12 bit that's what HAL says in documentation, but could be wrong
    float volt_drop_across_thermistor = (raw_data * THERMISTOR_V_1) / 4095.0f; // 2^12 - 1= 4095 (12 bit string  )

    // Circuit math to get temperature from voltage
    // Basically, the circuit looks like the following:
    // 3v3 -> resistor -> nucleo probe point -> thermistor -> ground.
    // The Nucleo's measured voltage is equal to voltage drop across
    // the resistor.
    // Since current is same, the V=IR implies that volt_therm/resistance_therm =
    // volt_resistor/resistance_resistor.
    // Thus, resistance_therm = volt_therm * resistance_resistor / volt_resistor.
    float volt_drop_across_resistor = THERMISTOR_V_1 - volt_drop_across_thermistor;
    float R_t = (THERMISTOR_RESISTOR_OHMS * volt_drop_across_thermistor) / (volt_drop_across_resistor);

    uint8_t const_set = 0;
    if (R_t < 692600.0f && R_t >= 32770.0f){
        const_set = 0;
    } else if (R_t < 32770.0f && R_t >= 3599.0f){
        const_set = 1;
    } else if (R_t < 3599.0f && R_t >= 681.6f){
        const_set = 2;
    } else if (R_t < 681.6f && R_t >= 187.0f){
        const_set = 3;
    } else {
        // Error out cause OOB temp
    	return;
    }
    float lnR_t_over_R_25 = log(R_t / THERMISTOR_R_25);

    float one_over_T = constant_array[const_set][0] + (constant_array[const_set][1] * lnR_t_over_R_25)
                     + (constant_array[const_set][2] * lnR_t_over_R_25 * lnR_t_over_R_25)
                     + (constant_array[const_set][3] * lnR_t_over_R_25 * lnR_t_over_R_25 * lnR_t_over_R_25);
    therm->temperature = (1 / one_over_T) - 273.15f;
}

// REQUIRES: thermistor is a Thermistor object
// MODIFIES: nothing
// EFFECTS: Get temperature of thermistor in degrees Celsius
float get_thermistor_temperature(Thermistor* therm) {
	return therm->temperature;
}
