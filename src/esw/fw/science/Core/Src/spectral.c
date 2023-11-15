#include "spectral.h"
#define I2C_AS72XX_SLAVE_STATUS_REG 0x00
#define I2C_AS72XX_SLAVE_TX_VALID 0x02

bool failed = false;
    

// REQUIRES: SMBus declared by user
// MODIFIES: nothing
// EFFECTS: Returns a pointer to a created Spectral object
Spectral *new_spectral(SMBus *smbus){
    Spectral* spectral = malloc(sizeof(Spectral));
    spectral->smbus = smbus;
    return spectral;
}


// REQUIRES: spectral is a Spectral object
// MODIFIES: nothing
// EFFECTS: Initializes the spectral device
void initialize_spectral(Spectral *spectral){    
    smbus_reset(spectral->smbus);
}

//OUR FUNCTIONS :)

//REQUIRES: spectral is a Spectral device
//MODIFIES: spectral
//EFFECTS: spectral channel_data is loaded with data read in from the sensor
void spectral_read(Spectral* spectral){

	//addresses for channel data
	char addresses[6] = {0x08, 0x09, 0x10, 0x11, 0x12, 0x13};
	const char DEV_ADDR = 0x49;

    //check if it is ok to write (see page 20 of the spectral sensor datasheet)
    volatile uint8_t status = smbus_read_byte_data(spectral->smbus, I2C_AS72XX_SLAVE_STATUS_REG, DEV_ADDR);
    if ((status & I2C_AS72XX_SLAVE_TX_VALID) != 0){
        failed = true;
        return;		
	}

	for (int i = 0; i < 6; ++i){
		spectral->channel_data[i] = smbus_read_byte_data(spectral->smbus, addresses[i], DEV_ADDR);
	}
	
}

//REQUIRES: spectral is a Spectral device
//MODIFIES: spectral
//EFFECTS: resets the spectral sensor
void mux_reset(I2C_HandleTypeDef* i2c_mux){
    SMBus* mux_bus = new_smbus(i2c_mux);
    smbus_write_byte_data(mux_bus, 0x00, 0x00, 0x70);
}

// REQUIRES: spectral is a Spectral object and 0 <= channel < 6
// MODIFIES: nothing
// EFFECTS: Returns the spectral data of a particular channel
uint16_t get_spectral_channel_data(Spectral *spectral, uint8_t channel){
	return spectral->channel_data;
}

/*REQUIRES: i2c_mux0 and 1 are valid GPIO pins, i2c_mux_pins are valid pins, and 
spectral_sensor_number is the pin number of the sensor to be set as the active I2C device
spectral_sensor_number is 0, 1, or 2 for B0, B1, or B2 
*/
//MODIFIES: mux pins are set so that the chosen spectral sensor is enabled.
//EFFECTS: spectral sensor chosen will now receive I2C signals
void set_active_spectral_sensor(I2C_HandleTypeDef* i2c_mux, int spectral_sensor_number) {

    if (spectral_sensor_number > 2 || spectral_sensor_number < 0){
        failed = true;
        return;
    }

    SMBus* bus = new_smbus(i2c_mux);
    uint8_t formatted = 1 << spectral_sensor_number;
    smbus_write_byte_data(bus, 0x70 << 1, spectral_sensor_number, 0x70);

    return;
}

