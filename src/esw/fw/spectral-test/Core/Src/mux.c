#include "mux.h"
#include "smbus.h"

int channel_map[8] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80 };
int spectral_exists = 0;
int triad_exists = 0;

Mux *new_mux(SMBus *i2c_bus) {
    Mux *mux = malloc(sizeof(Mux));
    mux->i2c_bus = i2c_bus;
    for (int i = 0; i < 8; ++i) {
        mux->channel_list[i] = 0x00;
    }
    mux->channels_active = 0;

    return mux;
}

void add_channel(Mux *mux, int channel) {
    if (channel > 7) {
        return;
    }
    mux->channel_list[channel] = channel_map[channel];
    mux->channels_active += 1;
}

void channel_select(Mux *mux, int channel){
    write_byte_data(mux->i2c_bus, I2C_MUX_ADDRESS, MUX_CMD, channel);
}

void del_mux(Mux *mux) {
	free(mux->channel_list);
	free(mux);
}
