#include "encoders.hpp"

#include <cstdint>

#include <units/units.hpp>

namespace mrover { 

    QuadratureEncoderReader::QuadratureEncoderReader(TIM_HandleTypeDef* timer, Ratio multiplier)
        : m_timer{timer}, m_multiplier{multiplier} {

        __HAL_TIM_SetCounter(m_timer, 0);
        // TODO(joseph) this seems really sussy and weird I'd like a better way of making sure it starts at zero
        m_counts_unwrapped_prev = __HAL_TIM_GetAutoreload(m_timer) / 2;
        check(HAL_TIM_Encoder_Start(m_timer, TIM_CHANNEL_ALL) == HAL_OK, Error_Handler);
    }

    std::int64_t QuadratureEncoderReader::count_delta() {
        m_counts_raw_now = m_timer->Instance->CNT;
        std::uint32_t max_value = m_timer->Instance->ARR;

        // adapted from: https://electronics.stackexchange.com/questions/605278/how-to-increase-stm32-timer-encoder-mode-counter-value
        // handle when timer wraps around
        std::int64_t c64 = static_cast<std::int64_t>(m_counts_raw_now) - max_value / 2; // remove half period to determine (+/-) sign of the wrap
        std::int64_t dif = c64 - m_counts_unwrapped_prev;                               // prev + (current - prev) = current

        // wrap difference from -HALF_PERIOD to HALF_PERIOD. modulo prevents differences after the wrap from having an incorrect result
        std::int64_t mod_dif = (dif + max_value / 2) % max_value - max_value / 2;
        if (dif < -max_value / 2) mod_dif += max_value; // account for mod of negative number behavior in C

        std::int64_t unwrapped = m_counts_unwrapped_prev + mod_dif;
        m_counts_unwrapped_prev = unwrapped;

        return mod_dif;
    }

    [[nodiscard]] auto QuadratureEncoderReader::read() -> std::optional<EncoderReading> {
        Radians delta_angle = m_multiplier * Ticks{count_delta()} / RELATIVE_CPR;
        m_ticks_now = HAL_GetTick();

        m_position += delta_angle;
        m_velocity = delta_angle / Seconds{1 / 10000.0f};

        m_ticks_prev = m_ticks_now;

        return EncoderReading{m_position, m_velocity};
    }

} // namespace mrover
