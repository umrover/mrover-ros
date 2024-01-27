#include "encoders.hpp"

#include <cstdint>

#include <units/units.hpp>

namespace mrover { 

    QuadratureEncoderReader::QuadratureEncoderReader(TIM_HandleTypeDef* timer, Ratio multiplier, TIM_HandleTypeDef* vel_timer)
        : m_timer{timer}, m_vel_timer{vel_timer}, m_multiplier{multiplier} {

        // Per Sashreek's hypothesis in `Motor velocity calc.pdf` #esw-brushed-24
        Seconds period = static_cast<float>(m_vel_timer->Instance->ARR) / CLOCK_FREQ;
        dt = Ticks{1} / RELATIVE_CPR / MIN_MEASURABLE_VELOCITY;
        // dt = Seconds{(1 / RELATIVE_CPR.get()) / MIN_MEASURABLE_VELOCITY.get()};
        Hertz fin = 1.0 / period;

        auto psc = static_cast<uint32_t>((fin * dt).get() / static_cast<float>(m_vel_timer->Instance->ARR + 1) - 1);
        m_vel_timer->Instance->PSC = psc; //static_cast<uint32_t>((fin * dt).get() / static_cast<float>(m_vel_timer->Instance->ARR + 1) - 1);

        // TODO(joseph) this seems really sussy and weird I'd like a better way of making sure it starts at zero
        m_counts_unwrapped_prev = __HAL_TIM_GetCounter(m_timer);
        check(HAL_TIM_Encoder_Start(m_timer, TIM_CHANNEL_ALL) == HAL_OK, Error_Handler);

        __HAL_TIM_SetCounter(m_vel_timer, 0);
        m_vel_counts_unwrapped_prev = __HAL_TIM_GetAutoreload(m_vel_timer) / 2;
        check(HAL_TIM_Base_Start_IT(m_vel_timer) == HAL_OK, Error_Handler);
    }

    auto count_delta(std::int64_t& store, TIM_HandleTypeDef* timer) -> std::int64_t {
        std::uint32_t now = timer->Instance->CNT;
        std::uint32_t max_value = timer->Instance->ARR;

        // adapted from: https://electronics.stackexchange.com/questions/605278/how-to-increase-stm32-timer-encoder-mode-counter-value
        // handle when timer wraps around
        std::int64_t c64 = static_cast<std::int64_t>(now) - max_value / 2; // remove half period to determine (+/-) sign of the wrap
        std::int64_t dif = c64 - store;                                    // prev + (current - prev) = current

        // wrap difference from -HALF_PERIOD to HALF_PERIOD. modulo prevents differences after the wrap from having an incorrect result
        std::int64_t mod_dif = (dif + max_value / 2) % max_value - max_value / 2;
        if (dif < -max_value / 2) mod_dif += max_value; // account for mod of negative number behavior in C

        std::int64_t unwrapped = store + mod_dif;
        store = unwrapped;

        return mod_dif;
    }

    [[nodiscard]] auto QuadratureEncoderReader::read() -> std::optional<EncoderReading> {
        int64_t delta_ticks = count_delta(m_counts_unwrapped_prev, m_timer);
        m_position += m_multiplier * Ticks{delta_ticks} / RELATIVE_CPR;
        return EncoderReading{m_position, m_velocity};
    }

    auto QuadratureEncoderReader::update_vel() -> void {
        const std::int64_t delta_ticks = count_delta(m_vel_counts_unwrapped_prev, m_vel_timer);
        const Radians delta_angle = m_multiplier * Ticks{delta_ticks} / RELATIVE_CPR;
        m_velocity = delta_angle / dt;
    }


} // namespace mrover
