#include "reader.hpp"

namespace mrover {

    QuadratureEncoder::QuadratureEncoder(TIM_TypeDef* _tim) : m_timer{_tim} {
    }

    int64_t QuadratureEncoder::count_delta() {
        m_counts_raw_now = m_timer->CNT;
        int64_t delta = m_counts_raw_now - m_counts_raw_prev;
        m_counts_raw_prev = m_counts_raw_now;
        return delta;
    }

} // namespace mrover
