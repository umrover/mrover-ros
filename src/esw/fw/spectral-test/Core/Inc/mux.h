#ifndef MUX_H_
#define MUX_H_

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f3xx_hal.h"
#include "smbus.h"


enum {
	I2C_MUX_ADDRESS = 0x70,
	MUX_CMD = 0xCC // filler address
};

extern int channel_map[8];
extern int spectral_exists;
extern int triad_exists;

typedef struct {
    SMBus *i2c_bus;
    int channel_list[8];
    int channels_active;
} Mux;

// takes in the i2c bus and the channels to be written 
// to on the mux (0 - 7), and the number of channels 
Mux *new_mux(SMBus *i2c_bus);

void add_channel(Mux *mux, int channel);

// Requires : channel is an actual register ie 0x01 (0x00 is not one of these)
// IF you want to read data, talk to only one sensor at a time -
// ie 0x01 talks to 0 only, 0x02 talks to 1 only,
// 0x04 talks to 2 only... 0x40 talks to 8 only
// IF you want to write to multiple rgb sensors you can
// "combine" the above ie 0x02 + 0x01 = 0x03 will write to 0 AND 1.
// 0xff will write to ALL rgb sensors
// Modifies which channel you are talking to

void channel_select(Mux *mux, int channel);

void del_mux(Mux *mux);

#endif
