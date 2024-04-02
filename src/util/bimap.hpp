#pragma once

#include <unordered_map>

namespace mrover {

    template<typename T>
    using optional_ref = std::optional<std::reference_wrapper<T>>;

    template<typename T, typename U, typename THash = std::hash<T>, typename UHash = std::hash<U>>
    class bimap {
        std::unordered_map<T, U, THash> m_forward;
        std::unordered_map<U, T, UHash> m_reverse;

    public:
        bimap() = default;

        bimap(std::initializer_list<std::pair<T, U>> const& list) {
            for (auto const& [t, u]: list) emplace(t, u);
        }

        void emplace(T const& t, U const& u) {
            m_forward.emplace(t, u);
            m_reverse.emplace(u, t);
        }

        optional_ref<U> forward(T const& t) {
            auto it = m_forward.find(t);
            return it == m_forward.end() ? std::nullopt : std::make_optional(std::ref(it->second));
        }

        optional_ref<T> backward(U const& u) {
            auto it = m_reverse.find(u);
            return it == m_reverse.end() ? std::nullopt : std::make_optional(std::ref(it->second));
        }
    };

} // namespace mrover
