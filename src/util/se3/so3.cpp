#include "so3.hpp"

SO3::SO3() : mQuaternion(Quaternion::Identity()) {}

SO3::SO3(Quaternion const& quaternion) : mQuaternion(quaternion) {
    mQuaternion.normalize();
}

SO3 SO3::fromMatrix(RotationMatrix const& rotationMatrix) {
    return SO3{Quaternion(rotationMatrix)};
}

SO3::Quaternion const& SO3::rotationQuaternion() const {
    return mQuaternion;
}

SO3::RotationMatrix SO3::rotationMatrix() const {
    return RotationMatrix(mQuaternion);
}

SO3::Vector SO3::directionVector() const {
    return rotationMatrix().col(0);
}

double SO3::rotDistanceTo(SO3 const& other) const {
    return mQuaternion.angularDistance(other.mQuaternion);
}

bool SO3::isApprox(SO3 const& other, double tolerance) const {
    return mQuaternion.isApprox(other.mQuaternion, tolerance);
}

SO3 SO3::identity() {
    return SO3{Quaternion::Identity()};
}
