#include "smbus.h"
#include "spectral.h"
#include "mux.h"

int spectral_channels[SPECTRAL_DEVICES] = { SPECTRAL_0_CHANNEL, SPECTRAL_1_CHANNEL, SPECTRAL_2_CHANNEL };
SMBus *i2c_bus;
Spectral *spectral;
uint16_t spectral_data[SPECTRAL_DEVICES * CHANNELS];
Mux *mux;

//transmits the spectral data as a sentance
//$SPECTRAL,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x,
//void send_spectral_data(uint16_t *data, UART_HandleTypeDef * huart){
//	int channels = 6;
//	int devices = SPECTRAL_DEVICES;
//
//	char string[158] = "";
//
//	uint8_t separated_data[36] = {0};
//
//	for (uint8_t i = 0; i < devices; ++i) {
//		//uint8_t start = i * (channels*6) + 10;
//		for (uint8_t j = 0; j < channels; ++j) {
//
//			//int temp = (i*2*channels) + (2*j) + 1;
//			uint8_t msb = (data[i*channels + j] >> 8) & 0xff;
//			uint8_t lsb = (data[i*channels + j]) & 0xff;
//
//			separated_data[(2*i*channels) + (2*j)] =  msb;
//			separated_data[(2*i*channels) + (2*j) + 1] =  lsb;
//
//			//sprintf((char *)string + start + j*6,",%u,%u", msb, lsb);
//		}
//	}
//
//	  sprintf((char *)string, "$SPECTRAL,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u, \r\n",\
//			  separated_data[0], separated_data[1], separated_data[2], separated_data[3], separated_data[4], separated_data[5], \
//			  separated_data[6], separated_data[7], separated_data[8], separated_data[9], separated_data[10], separated_data[11], \
//			  separated_data[12], separated_data[13], separated_data[14], separated_data[15], separated_data[16], separated_data[17], \
//			  separated_data[18], separated_data[19], separated_data[20], separated_data[21], separated_data[22], separated_data[23], \
//			  separated_data[24], separated_data[25], separated_data[26], separated_data[27], separated_data[28], separated_data[29], \
//			  separated_data[30], separated_data[31], separated_data[32], separated_data[33], separated_data[34], separated_data[35]);
//
//
//	//sprintf((char *)string + 10 + channels*6," \r\n");
//
//	// testing stuff - use if you don't have a
//	//char test[120];
//	//sprintf((char *)test, "$SPECTRAL,3,4,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,\n");
//
//	//HAL_UART_Transmit(huart, (uint8_t *)string, sizeof(string), 40);
//
//	HAL_UART_Transmit_IT(huart, (uint8_t *)string, sizeof(string));
//	HAL_Delay(40);
//}

// initalizes spectral object, adds bus to it
Spectral *new_spectral(SMBus *i2c_bus) {
    Spectral *spectral = malloc(sizeof(Spectral));
    spectral->i2c_bus = i2c_bus;

	uint8_t START_REG = 0x08; //RAW_VALUE_RGA_HIGH; *** TODO FIX THIS IS THE HIGH ONE

	for (uint8_t i = 0; i < CHANNELS; ++i) {
		spectral->channels[i] = _new_channel(START_REG + (2 * i), START_REG + (2 * i) + 1);
	}
	return spectral;
}

// sets enable bits in devices
void enable_spectral(Spectral *spectral) {
    _virtual_write(spectral, CONTROL_SET_UP, 0x28);  // runs twice to account for status miss
    HAL_Delay(5);
    _virtual_write(spectral, CONTROL_SET_UP, 0x28);  // converts data bank to 2
    _virtual_write(spectral, INT_TIME, 0xFF);  // increases integration time
}


// gets the data as an array of 16 bit integers
int get_spectral_data(Spectral *spectral, uint16_t *data) {

	 int error = _get_channel_data(spectral);
	 for (uint8_t i = 0; i < CHANNELS; ++i) {
		 data[i] = spectral->channels[i]->color_data;
	 }
	 if (error) {
		return 1;
	 }
	return 0;
}

void del_spectral(Spectral *spectral) {
	for (int i = 0; i < CHANNELS; ++i) {
		_del_channel(spectral->channels[i]);
	}
	free(spectral);
}


/*private interface*/

// functionallly like write_byte  
void _virtual_write(Spectral *spectral, uint8_t v_reg, uint8_t data) {
    uint8_t status;

	while(1) {
		status = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}
	write_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, (v_reg | 1 << 7));

	while(1) {
		status = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5);
	}
	write_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, data);
}

// functionally like read_byte
uint8_t _virtual_read(Spectral *spectral, uint8_t v_reg) {
    uint8_t status;
	uint8_t d;
	status = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);

	if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
		// d = nucleo_byte_read(I2C_AS72XX_SLAVE_READ_REG);
		d = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_READ_REG);
	}

	while(1) {
		status = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) {
			break;
		}
		HAL_Delay(5); //delay for 5 ms
	}

	write_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_WRITE_REG, v_reg);
	while(1) {
		status = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
			break;
		}
		HAL_Delay(5); //delay for 5 ms
	}

	d = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_READ_REG);
	return d;
}

int _get_channel_data(Spectral *spectral) {
    for (uint8_t i = 0; i < CHANNELS; ++i) {
        Channel *temp = spectral->channels[i];
        temp->color_data = _get_val(spectral, temp->lsb_register, temp->msb_register);
        if (spectral->i2c_bus->ret != HAL_OK) {
        	return 1;
        }
    }
    return 0;
}

// creates a channel
Channel* _new_channel(uint8_t msb_r, uint8_t lsb_r) {
    Channel* ch = malloc(sizeof(Channel));
	ch->color_data = 0;
	ch->lsb_register = lsb_r;
	ch->msb_register = msb_r;
	return ch;
}

// gets value of channel 
uint16_t _read_channel(Spectral *spectral, int channel) {
    return spectral->channels[channel]->color_data;
}

uint16_t _get_val(Spectral *spectral, uint8_t virtual_reg_l, uint8_t virtual_reg_h) {
    uint16_t high = (_virtual_read(spectral, virtual_reg_h) & 0xFF) << 8;
    return high | (_virtual_read(spectral, virtual_reg_l) & 0xFF);
}

void _del_channel(Channel *channel) {
	free(channel);
}

// Sets up all channels
void initialize_spectral() {
	// opens all channels on the mux to listen
	for (int i = 0; i < SPECTRAL_DEVICES; ++i) {
		channel_select(mux, mux->channel_list[i]);
		enable_spectral(spectral);
	}
}

void initialize_spectral_mux() {
	for (int i = 0; i < SPECTRAL_DEVICES; ++i) {
			add_channel(mux, spectral_channels[i]);
	}
}
// Check if spectral device is ready for data read
// Assumes the channel is already selected on the mux
// 1 = Not Ready
// 0 = Ready
int check_ready(){
	uint8_t status;
	status = read_byte_data(spectral->i2c_bus, DEVICE_SLAVE_ADDRESS, I2C_AS72XX_SLAVE_STATUS_REG);
	// If not ready, skip it
	if ((status & I2C_AS72XX_SLAVE_RX_VALID) != 0) {
		// d = nucleo_byte_read(I2C_AS72XX_SLAVE_READ_REG);
		return 0;
	}
	return 1;
}
