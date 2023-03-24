#include "se3.hpp"

Eigen::Quaterniond SO3::quaternion() const {
    return Eigen::Quaterniond{mAngleAxis};
}

Eigen::Matrix4d SO3::matrix() const {
    Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
    matrix.block<3, 3>(0, 0) = mAngleAxis.toRotationMatrix();
    return matrix;
}

SO3 SO3::operator*(SO3 const& other) const {
    return quaternion() * other.quaternion();
}
