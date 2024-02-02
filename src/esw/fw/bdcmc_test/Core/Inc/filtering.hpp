#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <numeric>

namespace mrover {

    template<IsUnit T, std::size_t Size>
    class RunningMeanFilter {

        std::size_t m_index{};
        T m_running_sum{};
        std::array<T, Size> m_circular_buffer{};

    public:
        auto add_reading(T reading) -> void {
            m_running_sum += reading - std::exchange(m_circular_buffer[m_index], reading);
            m_index = (m_index + 1) % Size;
        }

        auto clear() -> void {
            m_circular_buffer.fill(T{});
        }

        [[nodiscard]] auto get_filtered() const -> T {
            return std::reduce(m_circular_buffer.begin(), m_circular_buffer.end()) / Size;
        }
    };

    template<IsUnit T>
    class IIRFilter {

        T m_value{};
        double m_alpha{};

    public:
        IIRFilter(double alpha) : m_alpha{alpha} {}

        auto add_reaidng(T reading) -> void {
            m_value = m_alpha * reading + (1 - m_alpha) * m_value;
        }

        [[nodiscard]] auto get_filtered() const -> T {
            return m_value;
        }

        auto clear() -> void {
            m_value = T{};
        }
    };

    template<IsUnit T>
    class ButterworthFilter {

        
    };

} // namespace mrover
