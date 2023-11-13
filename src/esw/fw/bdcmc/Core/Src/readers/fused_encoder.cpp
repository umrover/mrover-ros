#include "reader.hpp"

#include "units/units.hpp"

#include "stm32g4xx_hal.h"

namespace mrover {

    FusedReader::FusedReader(TIM_TypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c, Ratio quad_multiplier, Ratio abs_multiplier)
            : m_relative_encoder_timer{relative_encoder_timer},
              m_absolute_encoder_i2c{absolute_encoder_i2c},
              m_abs_encoder{SMBus{absolute_encoder_i2c}, 0, 0, abs_multiplier},
              m_quad_encoder{TIM1, quad_multiplier} {

        // Initialize the TIM and I2C encoders
        check(HAL_I2C_Init(m_absolute_encoder_i2c) == HAL_OK, Error_Handler);

        // TODO - TIMERS need to be intiialized
//        check(HAL_TIM_Encoder_Start(m_relative_encoder_timer, TIM_CHANNEL_ALL) == HAL_OK, Error_Handler);
    }

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
