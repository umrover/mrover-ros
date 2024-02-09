#pragma once

#include <units/units.hpp>

namespace mrover {

    auto convertLinVel(float velocity, float multiplier) {
        return velocity * multiplier;
    }

    auto convertLinPos(Meters position, RadiansPerMeter multiplier) {
        return Radians{position * multiplier};
    }
} // namespace mrover
