#include "reader.hpp"

#include "units/units.hpp"

#include "stm32g4xx_hal.h"

namespace mrover {

    FusedReader::FusedReader(TIM_HandleTypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c, Ratio quad_multiplier, Ratio abs_multiplier)
        : m_abs_encoder{SMBus{absolute_encoder_i2c}, 0, 0, abs_multiplier},
          m_quad_encoder{relative_encoder_timer, quad_multiplier} {}

    void FusedReader::refresh_absolute() {

        // Set position to absolute if there is a valid reading and it has changed (rising edge)
        std::optional<std::uint64_t> count_from_absolute_encoder = m_abs_encoder.read_raw_angle();
        if (!count_from_absolute_encoder) return;

        m_position = RADIANS_PER_COUNT_ABSOLUTE * count_from_absolute_encoder.value();
    }

    [[nodiscard]] std::optional<EncoderReading> FusedReader::read() {
        // TODO - fix everything
        //		m_position += RADIANS_PER_COUNT_RELATIVE * m_quad_encoder.count_delta();
        // TODO update velocity here


        return std::make_optional(EncoderReading{m_position, m_velocity});
    }


} // namespace mrover
