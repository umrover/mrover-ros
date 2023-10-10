#include "smbus.h"	// for SMBus

// datasheet for spectral sensor: https://www.mouser.com/datasheet/2/588/AS7262_DS000486_2-00-1082195.pdf

// AS7262 Spectral sensor
typedef struct
{
	SMBus *smbus;
	uint16_t channel_data[SPECTRAL_CHANNELS];
} Spectral;

Spectral *new_spectral(SMBus *smbus);


// REQUIRES: SMBus declared by user
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Spectral object
Spectral *new_spectral(SMBus *smbus);


// REQUIRES: spectral is a Spectral object
// MODIFIES: nothing
// EFFECTS: Initializes the spectral device
void initialize_spectral(Spectral *spectral);
