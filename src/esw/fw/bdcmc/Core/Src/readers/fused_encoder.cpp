#include "reader.hpp"

#include "units.hpp"

#include "stm32g4xx_hal.h"

namespace mrover {

    FusedReader::FusedReader(TIM_HandleTypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c, Config const& config)
        : m_relative_encoder_timer{relative_encoder_timer}, m_absolute_encoder_i2c{absolute_encoder_i2c}, m_encoder_info{config.get_encoder_info()} {

        m_abs_encoder = AbsoluteEncoder{SMBus{absolute_encoder_i2c}, 1, /* TODO: figure out how to get _A1 */ 1 /* TODO: figure out how to get _A2 */};
        m_quad_encoder = QuadratureEncoder{TIM1};

        // Initialize the TIM and I2C encoders
        check(HAL_I2C_Init(m_absolute_encoder_i2c) == HAL_OK, Error_Handler);
        check(HAL_TIM_Encoder_Start(m_relative_encoder_timer, TIM_CHANNEL_ALL) == HAL_OK, Error_Handler);

        // Store state to center the relative encoder readings based on absolute
        refresh_absolute();
    }

    void FusedReader::refresh_absolute() {
        if (m_encoder_info.abs_present) {
            // Set position to absolute if there is a valid reading and it has changed (rising edge)
            std::optional<std::uint64_t> count_from_absolute_encoder = m_abs_encoder.read_raw_angle();
            if (!count_from_absolute_encoder) return;

            m_position = RADIANS_PER_COUNT_ABSOLUTE * count_from_absolute_encoder.value();
        }
    }

    [[nodiscard]] std::pair<Radians, RadiansPerSecond> FusedReader::read(Config const& config) {
        refresh_absolute();
        if (m_encoder_info.quad_present) {
            m_position += RADIANS_PER_COUNT_RELATIVE * m_quad_encoder.count_delta();
        }
        // TODO update velocity here
        return {m_position, m_velocity};
    }
        
    void FusedReader::configure(Config const& config) {
        this->m_encoder_info = config.get_encoder_info();
        this->m_limit_switch_info_0 = config.get_limit_switch_0();
        this->m_limit_switch_info_1 = config.get_limit_switch_1();
        this->m_limit_switch_info_2 = config.get_limit_switch_2(); 
        setup_limit_switches();
    }

    void FusedReader::setup_limit_switches() {
        if (m_limit_switch_info_0.a_present && m_limit_switch_info_0.a_enable) {
            m_limit_switch_a.enable();
        }
        m_limit_switch_info_2.

        if (m_limit_switch_info_0.b_present && m_limit_switch_info_0.b_enable) {
            m_limit_switch_b.enable();
        }

        if (m_limit_switch_info_0.c_present && m_limit_switch_info_0.c_enable) {
            m_limit_switch_c.enable();
        }

        if (m_limit_switch_info_0.d_present &&  m_limit_switch_info_0.d_enable) {
            m_limit_switch_d.enable();
        }
    }

} // namespace mrover
