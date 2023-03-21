#include "se3.hpp"
#include <stdexcept>

SO3::SO3(Eigen::Quaterniond const& quaternion) : mQuaternion(quaternion) {
    if (std::fabs(mQuaternion.norm() - 1.0) > std::numeric_limits<double>::epsilon()) {
        throw std::invalid_argument("Quaternion is not normalized");
    }
}

Eigen::Matrix4d SO3::matrix() const {
    auto affine = Eigen::Affine3d::Identity();
    affine.rotate(mQuaternion);
    return affine.matrix();
}

Eigen::Quaterniond SO3::quaternion() const {
    return mQuaternion;
}
