#include "reader.hpp"

namespace mrover {

    std::optional<std::uint64_t> AbsoluteEncoder::read_raw_angle() {
        std::optional raw_data_optional = this->m_i2cBus.transact<std::uint8_t, std::uint16_t>(this->m_address, 0xFF);
        if (!raw_data_optional) return std::nullopt;

        std::uint16_t raw_data = raw_data_optional.value();
        if (raw_data == m_previous_raw_data) return std::nullopt;

        std::uint16_t angle_left = (raw_data >> 8) & 0xFF; // 0xFE
        std::uint16_t angle_right = raw_data & 0xFF;       // 0xFF
        std::uint16_t angle_left_modified = angle_left & 0x3F;
        std::uint16_t angle_raw = (angle_right << 6) | angle_left_modified;
        m_previous_raw_data = raw_data;
        return raw_data;
    }

    // A1/A2 is 1 if pin connected to power, 0 if pin connected to ground
    AbsoluteEncoder::AbsoluteEncoder(SMBus i2c_bus, std::uint8_t A1, std::uint8_t A2) {
        // could be put into member list if we use ternary
        if (A1 && A2) {
            this->m_address = device_slave_address_both_high;
        } else if (A1) {
            this->m_address = device_slave_address_a1_high;
        } else if (A2) {
            this->m_address = device_slave_address_a2_high;
        } else {
            this->m_address = device_slave_address_none_high;
        }
    }

} // namespace mrover
