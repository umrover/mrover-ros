#include "reader.hpp"

#include <units.hpp>

#include "main.h"
#include "stm32g4xx_hal.h"

#include "controller.hpp"


namespace mrover {

    EncoderReader::EncoderReader(TIM_HandleTypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c) {
        m_relative_encoder_timer = relative_encoder_timer;
        m_absolute_encoder_i2c = absolute_encoder_i2c;

        m_abs_encoder = AbsoluteEncoder(SMBus(absolute_encoder_i2c), 1, /* TODO: figure out how to get _A1 */ 1/* TODO: figure out how to get _A2 */ );
        m_quad_encoder = QuadEncoder(TIM1);

        // Initialize the TIM and I2C encoders
        check(HAL_I2C_Init(m_absolute_encoder_i2c) == HAL_OK, Error_Handler);
        check(HAL_TIM_Encoder_Start(m_relative_encoder_timer, TIM_CHANNEL_ALL) == HAL_OK, Error_Handler);

        // Store state to center the relative encoder readings based on absolute
        refresh_absolute();
    }

    void EncoderReader::refresh_absolute() {
        uint32_t count_from_absolute_encoder = read_absolute();
        // Get reading from absolute encoder
        position = RADIANS_PER_COUNT_ABSOLUTE * count_from_absolute_encoder;
    }

    uint64_t EncoderReader::read_absolute() {
        return m_abs_encoder.read_raw_angle();
    }

    void EncoderReader::update(const Config& config) {
        auto delta = m_quad_encoder.count_delta();
        position += RADIANS_PER_COUNT_RELATIVE * delta;
    }

    // This is really read position
    [[nodiscard]] std::pair<Radians, RadiansPerSecond> EncoderReader::read() {
        // TODO update velocity here
        return {position, velocity};
    }

}
