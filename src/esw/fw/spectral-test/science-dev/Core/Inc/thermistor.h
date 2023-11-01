#ifndef THERMISTOR_H_
#define THERMISTOR_H_

// A thermistor device
typedef struct {
	ADC_HandleTypeDef *adc;
	float resistance;
	uint8_t channel;
} Thermistor;

// REQUIRES: adc is the adc channel,
// resistance is the resistance of the resistor in ohms,
// and channel is the channel
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Thermistor object
Thermistor *new_thermistor(ADC_HandleTypeDef *_adc, float resistance, uint8_t channel);

// REQUIRES: thermistor is a Thermistor object
// MODIFIES: nothing
// EFFECTS: Initializes the thermistor by changing the ADC settings
void initialize_thermistor(Thermistor* thermistor);

// REQUIRES: thermistor is a Thermistor object
// MODIFIES: nothing
// EFFECTS: Returns temperature of thermistor in degrees Celsius
void get_thermistor_temperature(Thermistor* thermistor);


#endif

//#endif
