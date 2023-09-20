#pragma once

#include <numbers>
#include <units.hpp>

#include "stm32g4xx_hal.h"

constexpr uint32_t COUNTS_PER_ROTATION_RELATIVE = 4096;
constexpr uint32_t COUNTS_PER_ROTATION_ABSOLUTE = 1024;
constexpr auto RADIANS_PER_COUNT_RELATIVE = mrover::Radians{2 * std::numbers::pi} / COUNTS_PER_ROTATION_RELATIVE;
constexpr auto RADIANS_PER_COUNT_ABSOLUTE = mrover::Radians{2 * std::numbers::pi} / COUNTS_PER_ROTATION_ABSOLUTE;

namespace mrover {

    class EncoderReader {
    public:
        EncoderReader() = default;
        EncoderReader(TIM_HandleTypeDef* relative_encoder_timer, I2C_HandleTypeDef* absolute_encoder_i2c);
        void refresh_absolute();
        void update_count();
        [[nodiscard]] Radians read_input() const;

    private:
        TIM_HandleTypeDef* m_relative_encoder_timer{};
        I2C_HandleTypeDef* m_absolute_encoder_i2c{};
        Radians rotation{};
        Radians absolute_relative_diff{};
    };

} // namespace mrover
