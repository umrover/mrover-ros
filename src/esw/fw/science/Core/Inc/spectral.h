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

//OUR FUNCTIONS :)

//REQUIRES: spectral is a Spectral device
//MODIFIES: spectral
//EFFECTS: spectral channel_data is loaded with data read in from the sensor
void spectral_read(Spectral* spectral){
	//addresses for channel data
	char addresses[6] = {0x08, 0x09, 0x10, 0x11, 0x12, 0x13};
	
	const char DEV_ADDR = 0x49;
	for (int i = 0; i < 6; ++i){
		spectral->channel_data[i] = smbus_read_byte_data(spectral->smbus, addresses[i], DEV_ADDR);
	}
	
}

