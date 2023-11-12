#include "reader.hpp"

namespace mrover {

    QuadratureEncoderReader::QuadratureEncoderReader(TIM_TypeDef* _tim) : m_timer{_tim} {
    }

    std::int64_t QuadratureEncoderReader::count_delta() {
        m_counts_raw_now = m_timer->CNT;

        // adapted from: https://electronics.stackexchange.com/questions/605278/how-to-increase-stm32-timer-encoder-mode-counter-value
        // handle when timer wraps around
        std::int64_t c64 = (int64_t)m_counts_raw_now - m_timer->ARR/2; // remove half period to determine (+/-) sign of the wrap
		std::int64_t dif = (c64-m_counts_unwrapped_prev); // prev + (current - prev) = current

		// wrap difference from -HALF_PERIOD to HALF_PERIOD. modulo prevents differences after the wrap from having an incorrect result
		std::int64_t mod_dif = ((dif + m_timer->ARR/2) % m_timer->ARR) - m_timer->ARR/2;
		if(dif < -m_timer->ARR/2)
			mod_dif += m_timer->ARR; // account for mod of negative number behavior in C

		std::int64_t unwrapped = m_counts_unwrapped_prev + mod_dif;
		m_counts_unwrapped_prev = unwrapped;

        return mod_dif;
    }

    [[nodiscard]] std::optional<EncoderReading> QuadratureEncoderReader::read() {
    	Radians delta_angle = RADIANS_PER_COUNT_RELATIVE * count_delta();
    	m_ticks_now = HAL_GetTick();

    	m_position += delta_angle;
    	m_velocity = delta_angle / ((m_ticks_now - m_ticks_prev) * SECONDS_PER_TICK);

    	m_ticks_prev = m_ticks_now;

    	return std::make_optional(EncoderReading{m_position, m_velocity});
    }
} // namespace mrover
