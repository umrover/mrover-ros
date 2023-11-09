#include "reader.hpp"

#include "units/units.hpp"

#include "stm32g4xx_hal.h"

namespace mrover {

    FusedReader::FusedReader(TIM_HandleTypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c)
            : m_relative_encoder_timer{relative_encoder_timer},
              m_absolute_encoder_i2c{absolute_encoder_i2c},
              m_abs_encoder{SMBus{absolute_encoder_i2c}, 1, /* TODO: figure out how to get _A1 */ 1 /* TODO: figure out how to get _A2 */},
              m_quad_encoder{TIM1} {

        // Initialize the TIM and I2C encoders
        check(HAL_I2C_Init(m_absolute_encoder_i2c) == HAL_OK, Error_Handler);
        check(HAL_TIM_Encoder_Start(m_relative_encoder_timer, TIM_CHANNEL_ALL) == HAL_OK, Error_Handler);
    }

    void FusedReader::refresh_absolute(Config const& config) {
        if (!config.quad_abs_enc_info.abs_present) return;

        // Set position to absolute if there is a valid reading and it has changed (rising edge)
        std::optional<std::uint64_t> count_from_absolute_encoder = m_abs_encoder.read_raw_angle();
        if (!count_from_absolute_encoder) return;

        m_position = RADIANS_PER_COUNT_ABSOLUTE * count_from_absolute_encoder.value();
    }

    [[nodiscard]] std::pair<Radians, RadiansPerSecond> FusedReader::read(Config const& config) {
        refresh_absolute(config);
        if (config.quad_abs_enc_info.quad_present) {
            m_position += RADIANS_PER_COUNT_RELATIVE * m_quad_encoder.count_delta();
        }
        // TODO update velocity here
        return {m_position, m_velocity};
    }

    void FusedReader::setup_limit_switches(Config const& config) {
        if (config.limit_switch_info_0.a_present && config.limit_switch_info_0.a_enable) {
            m_limit_switch_a.enable();
        }

        if (config.limit_switch_info_0.b_present && config.limit_switch_info_0.b_enable) {
            m_limit_switch_b.enable();
        }

        if (config.limit_switch_info_0.c_present && config.limit_switch_info_0.c_enable) {
            m_limit_switch_c.enable();
        }

        if (config.limit_switch_info_0.d_present && config.limit_switch_info_0.d_enable) {
            m_limit_switch_d.enable();
        }
    }

} // namespace mrover
