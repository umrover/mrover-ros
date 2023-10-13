#include "reader.hpp"

#include <units.hpp>

#include "main.h"
#include "stm32g4xx_hal.h"

#include "controller.hpp"


extern TIM_HandleTypeDef* htim3;
extern I2C_HandleTypeDef* absolute_encoder_i2c;

namespace mrover {

    void EncoderReader::init(TIM_HandleTypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c) {

        m_relative_encoder_timer = relative_encoder_timer;
        m_absolute_encoder_i2c = absolute_encoder_i2c;
        m_abs_encoder.init(SMBus(absolute_encoder_i2c), 1, /* TODO: figure out how to get _A1 */ 1,/* TODO: figure out how to get _A2 */ )
        m_quad_encoder.init(relative_encoder_timer, TIM2 /* TODO: need to figure out how to get _tim */)
    }

    void AbsoluteEncoder::init(SMBus _i2cBus, uint8_t _A1, uint8_t _A2) {

        // Initialize the TIM and I2C encoders
        check(HAL_TIM_Encoder_Init(m_relative_encoder_timer, nullptr /* TODO: replace with config */) == HAL_OK, Error_Handler);
        check(HAL_I2C_Init(m_absolute_encoder_i2c) == HAL_OK, Error_Handler);
        check(HAL_TIM_Encoder_Start(m_relative_encoder_timer, TIM_CHANNEL_ALL) == HAL_OK, Error_Handler);
        // Store state to center the relative encoder readings based on absolute
        refresh_absolute();
    }

    void EncoderReader::refresh_absolute() {
        uint32_t count_from_absolute_encoder = read_absolute();
        // Get reading from absolute encoder
        rotation = RADIANS_PER_COUNT_ABSOLUTE * count_from_absolute_encoder;
        // Calculate how much ahead/behind the absolute encoder's reading is with respect to the relative count
        absolute_relative_diff = rotation - RADIANS_PER_COUNT_RELATIVE * m_relative_encoder_timer->Instance->CNT;
    }

    uint32_t EncoderReader::read_absolute() {
        return m_abs_encoder.read_raw_angle();
    }

    void EncoderReader::update_count() {
        rotation = RADIANS_PER_COUNT_RELATIVE * m_relative_encoder_timer->Instance->CNT + absolute_relative_diff;
    }

    [[nodiscard]] Radians EncoderReader::read_input(const Config& config) const {
        return rotation;
    }

}
