#include "encoders.hpp"

#include <cstdint>
#include <optional>

namespace mrover {

    FusedReader::FusedReader(TIM_HandleTypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c, Ratio quad_multiplier, Ratio abs_multiplier)
        : m_absolute_encoder{SMBus{absolute_encoder_i2c}, 0, 0, abs_multiplier},
          m_relative_encoder{relative_encoder_timer, quad_multiplier} {}

    void FusedReader::refresh_absolute() {

        // Set position to absolute if there is a valid reading and it has changed (rising edge)
        std::optional<std::uint64_t> count_from_absolute_encoder = m_absolute_encoder.try_read_raw_angle();
        if (!count_from_absolute_encoder) return;

        m_position = Ticks{count_from_absolute_encoder.value()} / ABSOLUTE_CPR;
    }

    [[nodiscard]] std::optional<EncoderReading> FusedReader::read() {
        // TODO - fix everything
        //		m_position += RADIANS_PER_COUNT_RELATIVE * m_quad_encoder.count_delta();
        // TODO update velocity here

        return EncoderReading{m_position, m_velocity};
    }


} // namespace mrover
