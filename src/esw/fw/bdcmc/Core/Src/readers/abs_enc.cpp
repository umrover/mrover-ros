#include "reader.hpp"

namespace mrover {

    AbsoluteEncoder::AbsoluteEncoder(SMBus i2cBus, uint8_t A1, uint8_t A2)
        : m_i2cBus(i2cBus) {
        // A1/A2 is 1 if pin connected to power, 0 if pin connected to ground
        // could be put into member list if we use ternary
        if (A1 && A2) {
            this->m_address = device_slave_address_both_power;
        } else if (A1) {
            this->m_address = device_slave_address_a1_power;
        } else if (A2) {
            this->m_address = device_slave_address_a2_power;
        } else {
            this->m_address = device_slave_address_none_power;
        }
    }

    uint64_t AbsoluteEncoder::read_raw_angle() {
        uint64_t raw_data = this->m_i2cBus.read_word_data(this->m_address, 0xFF);
        uint64_t angle_left = ( raw_data >> 8 ) & 0xFF; // 0xFE
        uint64_t angle_right = raw_data & 0xFF; // 0xFF
        uint64_t angle_left_modified = angle_left & 0x3F;
        uint64_t angle_raw = (angle_right << 6) | angle_left_modified;
        return angle_raw;
    }

}
