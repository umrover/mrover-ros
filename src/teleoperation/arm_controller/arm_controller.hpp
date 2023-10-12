#include "pch.hpp"

namespace mrover {

    void ik_callback(IK const& newIkTarget);

    std::array<double, 3> solve(Eigen::Vector2d const& target);

} // namespace mrover
