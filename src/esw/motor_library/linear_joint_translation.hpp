#pragma once

#include <units/units.hpp>

namespace mrover {

    auto inline convertLinVel(float velocity, float multiplier) {
        return velocity * multiplier;
    }

    auto inline convertLinPos(float position, float multiplier) {
        return position * multiplier;
    }

} // namespace mrover
