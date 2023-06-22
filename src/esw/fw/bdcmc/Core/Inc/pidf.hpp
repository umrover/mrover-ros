#pragma once

#include "stm32g4xx_hal.h"

#include "units.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

template<typename InputUnit, typename OutputUnit, typename TimeUnit>
struct PIDF {
private:
    using InputUnit_t = unit_t<InputUnit>;
    using OutputUnit_t = unit_t<OutputUnit>;
    using TimeUnit_t = unit_t<TimeUnit>;
    using TotalErrorUnit_t = unit_t<compound_unit<InputUnit, TimeUnit>>;
    // Settings
    unit_t<compound_unit<OutputUnit, inverse<InputUnit>>> m_p{};
    unit_t<compound_unit<OutputUnit, inverse<InputUnit>, inverse<TimeUnit>>> m_i{};
    unit_t<compound_unit<OutputUnit, inverse<compound_unit<InputUnit, inverse<TimeUnit>>>>> m_d{};
    unit_t<compound_unit<OutputUnit, inverse<InputUnit>>> m_f{};
    InputUnit_t m_dead_band{};
    std::pair<OutputUnit_t, OutputUnit_t> m_output_bound{};
    std::optional<std::pair<InputUnit_t, InputUnit_t>> m_input_bound;
    // State
    TotalErrorUnit_t m_total_error{};
    InputUnit_t m_last_error{};
    TimeUnit_t m_last_time{};

    OutputUnit_t calculate(InputUnit_t input, InputUnit_t target, TimeUnit_t dt) {
        InputUnit_t error = target - input;

        if (m_input_bound) {
            auto [in_min, in_max] = m_input_bound.value();
            if (fabs(error) > (in_max - in_min) / 2) {
                if (error > make_unit<InputUnit_t>(0)) {
                    error -= in_max - in_min;
                } else {
                    error += in_max - in_min;
                }
            }
        }

        auto [out_min, out_max] = m_output_bound;
        if (out_min < error * m_p && error * m_p < out_max) {
            m_total_error += error * dt;
        } else {
            m_total_error = make_unit<TotalErrorUnit_t>(0);
        }

        InputUnit_t error_for_p = fabs(error) < m_dead_band ? make_unit<InputUnit_t>(0) : error;
        OutputUnit_t result = m_p * error_for_p + m_i * m_total_error + m_d * (error - m_last_error) / dt + m_f * target;
        m_last_error = error;

        if (result < out_min) return out_min;
        if (result > out_max) return out_max;
        return result;
    }

public:
    OutputUnit_t calculate(InputUnit_t input, InputUnit_t target) {
        auto now = make_unit<TimeUnit_t>(0);
        TimeUnit_t dt = now - m_last_time;
        m_last_time = now;
        return calculate(input, target, now);
    }

    PIDF& with_p(double p) {
        m_p = p;
        return *this;
    }

    PIDF& with_i(double i) {
        m_i = i;
        return *this;
    }

    PIDF& with_d(double d) {
        m_d = d;
        return *this;
    }

    PIDF& with_f(double f) {
        m_f = f;
        return *this;
    }

    PIDF& with_dead_band(InputUnit dead_band) {
        m_dead_band = dead_band;
        return *this;
    }

    PIDF& with_input_bound(InputUnit min, InputUnit max) {
        m_input_bound = {min, max};
        return *this;
    }

    PIDF& with_output_bound(double min, double max) {
        m_output_bound = {min, max};
        return *this;
    }
};