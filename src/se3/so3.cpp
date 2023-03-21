#include "se3.hpp"

SO3::SO3(Eigen::Quaterniond const& quaternion) : mQuaternion(quaternion) {
}

Eigen::Matrix4d SO3::matrix() const {
    auto affine = Eigen::Affine3d::Identity();
    affine.rotate(mQuaternion);
    return affine.matrix();
}
