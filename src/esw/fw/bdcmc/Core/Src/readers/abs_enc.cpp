#include "reader.hpp"

namespace mrover {

    uint64_t AbsoluteEncoder::read_raw_angle() {
        int raw_data = this->m_i2cBus.read_word_data(this->m_address, 0xFF);
        int angle_left = ( raw_data >> 8 ) & 0xFF; // 0xFE
        int angle_right = raw_data & 0xFF; // 0xFF
        int angle_left_modified = angle_left & 0x3F;
        int angle_raw = (angle_right << 6) | angle_left_modified;
        return angle_raw;
    }

    // A1/A2 is 1 if pin connected to power, 0 if pin connected to ground
    AbsoluteEncoder::AbsoluteEncoder(SMBus _i2cBus, uint8_t _A1, uint8_t _A2) {
        // could be put into member list if we use ternary
        if (_A1 && _A2) {
            this->m_address = device_slave_address_both_high;
        } else if (_A1) {
            this->m_address = device_slave_address_a1_high;
        } else if (_A2) {
            this->m_address = device_slave_address_a2_high;
        } else {
            this->m_address = device_slave_address_none_high;
        }
    }

}
