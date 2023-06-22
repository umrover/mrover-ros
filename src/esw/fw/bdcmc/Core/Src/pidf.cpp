#include "pidf.hpp"

#include <algorithm>
#include <cmath>

double pidf::calculate(double input, double target, double dt) {
    double error = target - input;

    if (m_input_bound) {
        auto [in_min, in_max] = m_input_bound.value();
        if (std::fabs(error) > (in_max - in_min) / 2) {
            if (error > 0) {
                error -= in_max - in_min;
            } else {
                error += in_max - in_min;
            }
        }
    }

    auto [out_min, out_max] = m_output_bound;
    if (double ep = m_p * error; std::clamp(ep, out_min, out_max) == ep) {
        m_total_error += error * dt;
    } else {
        m_total_error = 0;
    }

    double p_error = std::fabs(error) < m_dead_band ? 0 : m_p * error;
    double result = m_p * p_error + m_i * m_total_error + m_d * (error - m_last_error) / dt + m_f * target;
    m_last_error = error;

    return std::clamp(result, out_min, out_max);
}

double pidf::calculate(double input, double target) {
    auto now = hr_clock::now();
    double dt = std::chrono::duration<double>(now - m_last).count();
    m_last = now;

    return calculate(input, target, dt);
}
