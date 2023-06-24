#pragma once

#include "stm32g4xx_hal.h"

#include "units.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <utility>

/**
 * A PIDF controller. An output signal is generated based on the error of the current input and the desired input target.
 *  P - Proportional    Multiplied by the error
 *  I - Integral        Multiplied by the integral of the error. Alleviates steady state error, use with caution.
 *  D - Derivative      Multiplied by the derivative of the error
 *  F - Feedforward     Multiplied by the target. This is useful for gravity compensation.
 *
 * @tparam TInput   Unit of input, usually a sensor reading (for example encoder ticks for an arm)
 * @tparam TOutput  Unit of output, usually a motor command (for example voltage for a motor)
 * @tparam TTime    Unit of time
 */
template<typename TInput, typename TOutput, typename TTime>
struct PIDF {
private:
    using Input = unit_t<TInput>;
    using Output = unit_t<TOutput>;
    using Time = unit_t<TTime>;
    using TotalError = unit_t<compound_unit<TInput, TTime>>;
    // Settings
    unit_t<compound_unit<TOutput, inverse<TInput>>> m_p{};
    unit_t<compound_unit<TOutput, inverse<TInput>, inverse<TTime>>> m_i{};
    unit_t<compound_unit<TOutput, inverse<compound_unit<TInput, inverse<TTime>>>>> m_d{};
    unit_t<compound_unit<TOutput, inverse<TInput>>> m_f{};
    Input m_dead_band{};
    std::pair<Output, Output> m_output_bound{};
    std::optional<std::pair<Input, Input>> m_input_bound;
    // State
    TotalError m_total_error{};
    Input m_last_error{};
    Time m_last_time{};

    Output calculate(Input input, Input target, Time dt) {
        Input error = target - input;

        if (m_input_bound) {
            auto [in_min, in_max] = m_input_bound.value();
            if (fabs(error) > (in_max - in_min) / 2) {
                if (error > make_unit<Input>(0)) {
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
            m_total_error = make_unit<TotalError>(0);
        }

        Input error_for_p = fabs(error) < m_dead_band ? make_unit<Input>(0) : error;
        Output result = m_p * error_for_p + m_i * m_total_error + m_d * (error - m_last_error) / dt + m_f * target;
        m_last_error = error;

        if (result < out_min) return out_min;
        if (result > out_max) return out_max;
        return result;
    }

public:
    /**
     * TODO: documentation
     *
     * @param input     Current value
     * @param target    Desired final value
     * @return          Output value to control the input to move to the target
     */
    Output calculate(Input input, Input target) {
        double current_ticks = HAL_GetTick();
        auto tick_frequency = make_unit<hertz_t>(HAL_GetTickFreq());
        Time now = current_ticks / tick_frequency;
        Time dt = now - m_last_time;
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

    PIDF& with_dead_band(TInput dead_band) {
        m_dead_band = dead_band;
        return *this;
    }

    PIDF& with_input_bound(TInput min, TInput max) {
        m_input_bound = {min, max};
        return *this;
    }

    PIDF& with_output_bound(double min, double max) {
        m_output_bound = {min, max};
        return *this;
    }
};