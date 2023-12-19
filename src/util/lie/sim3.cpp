#include "se3.hpp"

SIM3::SIM3(R3 const& position, SO3 const& rotation, R3 const& scale) {
    mTransform.translate(position);
    mTransform.rotate(rotation.mAngleAxis);
    mTransform.scale(scale);
}

Eigen::Matrix4d SIM3::matrix() const {
    return mTransform.matrix();
}
