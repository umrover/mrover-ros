#pragma once

#include <cstdint>

namespace mrover {

    struct Point {
        float x, y, z;
        uint8_t b, g, r, a;
        float normal_x, normal_y, normal_z;
        float curvature;
    } __attribute__((packed));

} // namespace mrover
