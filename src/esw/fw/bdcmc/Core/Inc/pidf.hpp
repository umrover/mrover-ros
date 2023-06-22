#pragma once

#include <chrono>
#include <optional>
#include <utility>

struct pidf {
private:
    using hr_clock = std::chrono::high_resolution_clock;
    // Settings
    double m_p{}, m_i{}, m_d{}, m_f{};
    double m_dead_band{};
    std::pair<double, double> m_output_bound = {-1, 1};
    std::optional<std::pair<double, double>> m_input_bound;
    // State
    double m_total_error{};
    double m_last_error{};
    hr_clock::time_point m_last = hr_clock::now();

    double calculate(double input, double target, double dt);

public:
    double calculate(double input, double target);

    pidf& with_p(double p) {
        m_p = p;
        return *this;
    }

    pidf& with_i(double i) {
        m_i = i;
        return *this;
    }

    pidf& with_d(double d) {
        m_d = d;
        return *this;
    }

    pidf& with_f(double f) {
        m_f = f;
        return *this;
    }

    pidf& with_dead_band(double dead_band) {
        m_dead_band = dead_band;
        return *this;
    }

    pidf& with_input_bound(double min, double max) {
        m_input_bound = {min, max};
        return *this;
    }

    pidf& with_output_bound(double min, double max) {
        m_output_bound = {min, max};
        return *this;
    }
};