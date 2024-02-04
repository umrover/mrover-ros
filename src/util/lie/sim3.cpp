#include "se3.hpp"

SIM3::SIM3(R3 const& position, SO3 const& rotation, R3 const& scale) {
    mTransform.fromPositionOrientationScale(position, rotation.mAngleAxis, scale);
}

auto SIM3::matrix() const -> Eigen::Matrix4d {
    return mTransform.matrix();
}

auto SIM3::position() const -> R3 {
    return mTransform.translation();
}
