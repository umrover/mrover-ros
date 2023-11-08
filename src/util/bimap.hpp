#include <unordered_map>

namespace mrover {

    template<typename T, typename U, typename THash = std::hash<T>, typename UHash = std::hash<U>>
    class bimap {
        std::unordered_map<T, U, THash> m_forward;
        std::unordered_map<U, T, UHash> m_reverse;

    public:
        void emplace(T const& t, U const& u) {
            m_forward.emplace(t, u);
            m_reverse.emplace(u, t);
        }

        bool contains(T const& t) const {
            return m_forward.contains(t);
        }

        U const& forward(T const& t) const {
            return m_forward.at(t);
        }

        T const& backward(U const& u) const {
            return m_reverse.at(u);
        }
    };

} // namespace mrover
