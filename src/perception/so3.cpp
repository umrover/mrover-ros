#include "so3.hpp"

SO3::SO3(Quaternion const& quaternion) : quaternion(quaternion) {
}

SO3 SO3::fromMatrix(RotationMatrix const& rotationMatrix) {
    return SO3{Quaternion(rotationMatrix)};
}

SO3::RotationMatrix SO3::rotationMatrix() {
    return RotationMatrix(quaternion);
}

SO3::Vector SO3::directionVector() {
    return rotationMatrix().col(0);
}

double SO3::rotDistanceTo(SO3 const& other) {
    return quaternion.angularDistance(other.quaternion);
}

bool SO3::isApprox(SO3 const& other, double tolerance) {
    return rotDistanceTo(other) < tolerance;
}

SO3 SO3::identity() {
    return SO3{Quaternion::Identity()};
}
