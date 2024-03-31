#include "encoders.hpp"

#include <cstdint>
#include <utility>

#include <units/units.hpp>

namespace mrover {

    QuadratureEncoderReader::QuadratureEncoderReader(TIM_HandleTypeDef* tick_timer, Ratio multiplier, TIM_HandleTypeDef* elapsed_timer)
        : m_tick_timer{tick_timer}, m_elapsed_timer{elapsed_timer}, m_multiplier{multiplier} {

        // // Per Sashreek's hypothesis in `Motor velocity calc.pdf` #esw-brushed-24
        // compound_unit<Ticks, inverse<Seconds>> min_measurable_velocity = MIN_MEASURABLE_VELOCITY * RELATIVE_CPR;
        // m_velocity_dt = Seconds{1 / min_measurable_velocity.get()};
        //
        // auto psc = static_cast<std::uint32_t>((CLOCK_FREQ * m_velocity_dt).get() / static_cast<float>(m_timer->Instance->ARR + 1) - 1);
        // m_timer->Instance->PSC = psc;

        m_counts_unwrapped_prev = __HAL_TIM_GetCounter(m_tick_timer);
        check(HAL_TIM_Encoder_Start_IT(m_tick_timer, TIM_CHANNEL_ALL) == HAL_OK, Error_Handler);

        // m_vel_counts_unwrapped_prev = __HAL_TIM_GetCounter(m_timer);
        // check(HAL_TIM_Base_Start_IT(m_timer) == HAL_OK, Error_Handler);

        check(HAL_TIM_Base_Start_IT(m_elapsed_timer) == HAL_OK, Error_Handler);
    }

    auto count_delta(std::int64_t& store, TIM_HandleTypeDef* timer) -> std::int64_t {
        std::uint32_t now = __HAL_TIM_GetCounter(timer);
        std::uint32_t max_value = __HAL_TIM_GetAutoreload(timer);

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

    [[nodiscard]] auto QuadratureEncoderReader::read() const -> std::optional<EncoderReading> {
        return std::make_optional<EncoderReading>(m_position, m_velocity_filter.get_filtered());
        // return std::make_optional<EncoderReading>(m_position, m_velocity);
    }

    auto QuadratureEncoderReader::update() -> void {
        std::uint32_t elapsed_count = std::exchange(__HAL_TIM_GetCounter(m_elapsed_timer), 0);
        std::int64_t delta_ticks = count_delta(m_counts_unwrapped_prev, m_tick_timer);

        // if (delta_ticks == 0 || elapsed_count == 0) return;

        Radians delta_angle = m_multiplier * Ticks{delta_ticks} / RELATIVE_CPR;
        Seconds elapsed_time = 1 / CLOCK_FREQ * elapsed_count;

        m_position += delta_angle;
        // m_velocity = delta_angle / elapsed_time;
        m_velocity_filter.add_reading(delta_angle / elapsed_time);
    }


} // namespace mrover
