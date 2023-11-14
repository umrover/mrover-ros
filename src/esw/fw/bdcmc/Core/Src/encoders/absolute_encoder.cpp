#include "encoders.hpp"

#include <cstdint>
#include <optional>

namespace mrover {

    // A1/A2 is 1 if pin connected to power, 0 if pin connected to ground
    AbsoluteEncoderReader::AbsoluteEncoderReader(SMBus i2c_bus, std::uint8_t A1, std::uint8_t A2, Ratio multiplier)
        : m_i2cBus{i2c_bus}, m_multiplier{multiplier} {
        // could be put into member list if we use ternary
        if (A1 && A2) {
            m_address = I2CAddress::device_slave_address_both_high;
        } else if (A1) {
            m_address = I2CAddress::device_slave_address_a1_high;
        } else if (A2) {
            m_address = I2CAddress::device_slave_address_a2_high;
        } else {
            m_address = I2CAddress::device_slave_address_none_high;
        }
    }

    auto AbsoluteEncoderReader::try_read_raw_angle() -> std::optional<std::uint64_t> {
        std::optional raw_data_optional = m_i2cBus.transact<std::uint8_t, std::uint16_t>(m_address, 0xFF);
        if (!raw_data_optional) return std::nullopt;

        std::uint16_t raw_data = raw_data_optional.value();
        if (raw_data == m_previous_raw_data) return std::nullopt;

        std::uint16_t angle_left = raw_data >> 8 & 0xFF; // 0xFE
        std::uint16_t angle_right = raw_data & 0xFF;     // 0xFF
        std::uint16_t angle_left_modified = angle_left & 0x3F;
        std::uint16_t angle_raw = (angle_right << 6) | angle_left_modified;
        m_previous_raw_data = raw_data;
        return raw_data;
    }

    [[nodiscard]] auto AbsoluteEncoderReader::read() -> std::optional<EncoderReading> {
        if (std::optional<std::uint64_t> count = try_read_raw_angle()) {
            std::uint64_t ticks_now = HAL_GetTick();
            m_position = m_multiplier * Ticks{count.value()} / ABSOLUTE_CPR;
            Seconds seconds_per_tick = 1 / Hertz{HAL_GetTickFreq()};
            m_velocity = (m_position - m_angle_prev) / ((ticks_now - m_ticks_prev) * seconds_per_tick);
            m_ticks_prev = ticks_now;
        }

        return EncoderReading{m_position, m_velocity};
    }

} // namespace mrover
