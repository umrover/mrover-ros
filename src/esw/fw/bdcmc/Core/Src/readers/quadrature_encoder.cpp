#include "reader.hpp"

namespace mrover {

    QuadratureEncoder::QuadratureEncoder(TIM_TypeDef* _tim) : m_timer{_tim} {
    }

    int64_t QuadratureEncoder::count_delta() {
        this->m_counts_raw_now = this->m_timer->CNT;
        int64_t delta = this->m_counts_raw_now - this->m_counts_raw_prev;
        this->m_counts_raw_prev = this->m_counts_raw_now;
        return delta;
    }

} // namespace mrover
