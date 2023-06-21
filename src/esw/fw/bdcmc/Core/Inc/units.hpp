#pragma once

#include <ratio>

template<typename Rep, typename Period>
struct unit_t {
public:
    unit_t(Rep value) : m_value{value} {}

    // TODO: Add operators
private:
    Rep m_value;
};

using meters = unit_t<double, std::ratio<1>>;

using meters_per_second = unit_t<double, std::ratio<1>>;

meters operator"" _m(long double value) {
    return meters{static_cast<double>(value)};
}

meters_per_second operator"" _mps(long double value) {
    return meters_per_second{static_cast<double>(value)};
}
