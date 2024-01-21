#include "se3.hpp"

Eigen::Quaterniond SO3::quaternion() const {
    return Eigen::Quaterniond{mAngleAxis};
}

Eigen::Matrix3d SO3::matrix() const {
    return mAngleAxis.toRotationMatrix();
}

SO3 SO3::operator*(SO3 const& other) const {
    return other.quaternion() * quaternion();
}

R3 SO3::operator*(R3 const& other) const {
    return quaternion() * other;
}
