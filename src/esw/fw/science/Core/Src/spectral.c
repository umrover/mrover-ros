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
// REQUIRES: spectral is a Spectral object and 0 <= channel < 6
// MODIFIES: nothing
// EFFECTS: Returns the spectral data of a particular channel
uint16_t get_spectral_channel_data(Spectral *spectral, uint8_t channel){
	return spectral->channel_data;
}

//REQUIRES: sensors is an array of Spectral structs of size 3
//MODIFIES: nothing
//EFFECTS: returns a pointer to the spectral sensor that should be used
Spectral* get_active_spectral_sensor(GPIO_InitTypeDef i2c_mux0, int i2c_mux0_pin, 
    			GPIO_InitTypeDef i2c_mux1, int i2c_mux1_pin, Spectral sensors[] ) {

	int pin1 = HAL_GPIO_ReadPin(i2c_mux0, i2c_mux0_pin);
    int pin2 = HAL_GPIO_ReadPin(i2c_mux1, i2c_mux1_pin);
    if (pin1 && pin2){
        return NULL;
    }
    return &(sensors[2 * pin1 + pin2]);
}


