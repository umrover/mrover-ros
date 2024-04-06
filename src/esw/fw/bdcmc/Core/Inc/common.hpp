#pragma once

#include <units/units.hpp>

namespace mrover {

    constexpr auto TAU_F = 2 * std::numbers::pi_v<float>;

    constexpr auto CLOCK_FREQ = Hertz{140000000};

    // Counts (ticks) per radian (NOT per rotation)
    using CountsPerRad = compound_unit<Ticks, inverse<Radians>>;

    constexpr auto RELATIVE_CPR = CountsPerRad{3355 / TAU_F};      // Measured empirically from the devboard
    constexpr auto ABSOLUTE_CPR = CountsPerRad{(1 << 14) / TAU_F}; // Corresponds to the AS5048B

    // NOTE: Change This For Each Motor Controller
    constexpr static std::uint8_t DEVICE_ID = 0x21; // currently set for joint_b

    // Usually this is the Jetson
    constexpr static std::uint8_t DESTINATION_DEVICE_ID = 0x10;

} // namespace mrover
