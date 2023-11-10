#include "smbus.h"	// for SMBus
// datasheet for spectral sensor: https://www.mouser.com/datasheet/2/588/AS7262_DS000486_2-00-1082195.pdf

#define SPECTRAL_DEVICES 3
#define SPECTRAL_CHANNELS 6

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

//REQUIRES: spectral is a Spectral device
//MODIFIES: spectral
//EFFECTS: spectral channel_data is loaded with data read in from the sensor
void spectral_read(Spectral* spectral);

//REQUIRES: spectral is a Spectral device, buffer is an array of data to write
//MODIFIES: spectral
//EFFECTS: writes the provided data to the sensor 
void spectral_write(Spectral* spectral, uint8_t buffer[]);

// REQUIRES: spectral is a Spectral object and 0 <= channel < 6
// MODIFIES: nothing
// EFFECTS: Returns the spectral data of a particular channel
uint16_t get_spectral_channel_data(Spectral *spectral, uint8_t channel);
//REQUIRES: sensors is an array of Spectral structs of size 3
//MODIFIES: nothing
//EFFECTS: returns a pointer to the spectral sensor that should be used
Spectral* get_active_spectral_sensor(GPIO_InitTypeDef i2c_mux0, int i2c_mux0_pin, 
    			GPIO_InitTypeDef i2c_mux1, int i2c_mux1_pin, Spectral sensors[] );


// TODO: add virtual read and write functions (find information from datasheet)
