#include "encoders.hpp"

#include <cstdint>
#include <optional>

namespace mrover {
    /* ABSOLUTE ENCODER PROCESS:
    1. ABSOLUTE_ENCODER_TIMER elapses
    2. Request transmission sent
    3. Transmission callback interrupt
    4. Set up read
    5. Reception callback interrupt
    6. Update controller
    */

    // A1/A2 is 1 if pin connected to power, 0 if pin connected to ground
    AbsoluteEncoderReader::AbsoluteEncoderReader(AS5048B_Bus i2c_bus, std::uint8_t A1, std::uint8_t A2, Ratio multiplier, TIM_HandleTypeDef* elapsed_timer)
        : m_i2cBus{i2c_bus}, m_multiplier{multiplier}, m_elapsed_timer{elapsed_timer} {
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

        // Start elapsed timer
        check(HAL_TIM_Base_Start_IT(m_elapsed_timer) == HAL_OK, Error_Handler);
    }

    auto AbsoluteEncoderReader::request_raw_angle() -> void {
        m_i2cBus.async_transmit(m_address, 0xFE);
    }

    auto AbsoluteEncoderReader::read_raw_angle_into_buffer() -> void {
        m_i2cBus.async_receive(m_address);
    }

    auto AbsoluteEncoderReader::try_read_buffer() -> std::optional<std::uint64_t> {
        std::optional raw_data_optional = m_i2cBus.get_buffer();
        if (!raw_data_optional) return std::nullopt;

        // union {
        // std::uint16_t raw_data;
        // std::uint8_t raw_data_bytes[2];
        // } meme;

        std::array<std::uint8_t, 2> raw_data = raw_data_optional.value();
        // if (meme.raw_data == m_previous_raw_data) return std::nullopt;

        std::uint16_t angle = raw_data[1] << 6 | raw_data[0] & 0x3F;

        // TODO(owen): Why is this unused? If it truly is, remove it
        // I have no clue -Owen
        // std::uint16_t angle_left = raw_data >> 8 & 0xFF; // 0xFE
        // std::uint16_t angle_right = raw_data & 0xFF;     // 0xFF
        // std::uint16_t angle_left_modified = angle_left & 0x3F;
        // std::uint16_t angle_raw = (angle_right << 6) | angle_left_modified;

        m_previous_raw_data = angle;
        return angle;
    }

    [[nodiscard]] auto AbsoluteEncoderReader::read() -> std::optional<EncoderReading> {
        if (std::optional<std::uint64_t> count = try_read_buffer()) {
            std::uint32_t elapsed_count = std::exchange(__HAL_TIM_GetCounter(m_elapsed_timer), 0);
            Seconds elapsed_time = 1 / CLOCK_FREQ * elapsed_count;

            m_position = m_multiplier * Ticks{count.value()} / ABSOLUTE_CPR;
            m_velocity_filter.add_reading((m_position - m_position_prev) / elapsed_time);
            m_position_prev = m_position;
        }

        return std::make_optional<EncoderReading>(m_position, m_velocity_filter.get_filtered());
    }

} // namespace mrover