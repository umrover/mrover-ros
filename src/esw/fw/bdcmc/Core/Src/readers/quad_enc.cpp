#include "controller.hpp"
#include "reader.hpp"

namespace mrover {

    QuadEncoder::QuadEncoder(TIM_TypeDef *_tim) : m_tim(_tim), m_counts_raw_prev{}, m_counts_raw_now{} {
    }

    int32_t QuadEncoder::count_delta() {
        this->m_counts_raw_now = this->m_tim->CNT;
        int32_t delta = this->m_counts_raw_now - this->m_counts_raw_prev;
        this->m_counts_raw_prev = this->m_counts_raw_now;
        return delta;
    }

}
