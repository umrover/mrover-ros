#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <numeric>

namespace mrover {

    template<IsUnit T, std::size_t Size>
        requires(Size % 2 == 1)
    class MeanMedianFilter {

        std::size_t m_index{};
        std::array<T, Size> m_circular_buffer{};

    public:
        auto add_reading(T reading) -> void {
            m_circular_buffer[m_index] = reading;
            m_index = (m_index + 1) % Size;
        }

        [[nodiscard]] auto get_filtered() const -> T {
            std::array<T, Size> sorted_buffer = m_circular_buffer;
            std::ranges::sort(sorted_buffer, std::less{});
            auto begin = sorted_buffer.begin() + Size / 4;
            auto end = sorted_buffer.end() - Size / 4;
            return std::reduce(begin, end) / std::distance(begin, end);
        }
    };

} // namespace mrover
