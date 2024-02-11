#pragma once

#include <units/units.hpp>

namespace mrover {

    auto inline convertLinVel(float velocity, float multiplier) {
        return velocity * multiplier;
    }

    auto inline convertLinPos(Meters position, RadiansPerMeter multiplier) {
        return Radians{position * multiplier};
    }
} // namespace mrover
