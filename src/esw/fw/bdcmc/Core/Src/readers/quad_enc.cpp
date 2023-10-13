#include "reader.hpp"

namespace mrover {

    void QuadEncoder::update() {
        
    }

    void QuadEncoder::init(TIM_HandleTypeDef *_htim, TIM_TypeDef *_tim) {
        this->m_tim = _tim;
        this->m_htim = _htim;
        this->m_counts = 0;
        this->m_counts_raw_prev = 0;
        this->m_counts_raw_now = 0;
    }

    int32_t QuadEncoder::count_delta() {
        this->m_counts_raw_now = this->m_tim->CNT;
        int32_t delta = this->m_counts_raw_now - this->m_counts_raw_prev;
        this->m_counts_raw_prev = this->m_counts_raw_now;
        return delta;
    }

}
