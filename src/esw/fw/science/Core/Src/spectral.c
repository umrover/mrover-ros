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

/*REQUIRES: i2c_mux0 and 1 are valid GPIO pins, i2c_mux_pins are valid pins, and 
spectral_sensor_number is the number of the sensor to be turned on (0 - 2 inclusive)
Mux system (i2c_mux0_pin, i2c_mux1_pin) => Spectral sensor:
1, 1 => Not allowed
1, 0 => Spectral sensor 2
0, 1 => Spectral sensor 1
0, 0 => Spectral sensor 0
*/
//MODIFIES: mux pins are set so that the right spectral sensor is enabled.
//EFFECTS: None
Spectral* get_active_spectral_sensor(GPIO_InitTypeDef i2c_mux0, int i2c_mux0_pin, 
    			GPIO_InitTypeDef i2c_mux1, int i2c_mux1_pin, int spectral_sensor_number) {

    if (spectral_sensor_number > 2 || spectral_sensor_number < 0){
        failed = true;
        return;
    }

    if (spectral_sensor_number == 2){
        HAL_GPIO_WritePin(i2c_mux0, i2c_mux0_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(i2c_mux1, i2c_mux1_pin, GPIO_PIN_RESET);
        return;
    }

    if (spectral_sensor_number == 1){
        HAL_GPIO_WritePin(i2c_mux0, i2c_mux0_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(i2c_mux1, i2c_mux1_pin, GPIO_PIN_SET);
        return;
    }

    HAL_GPIO_WritePin(i2c_mux0, i2c_mux0_pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(i2c_mux1, i2c_mux1_pin, GPIO_PIN_RESET);
    return;

}


