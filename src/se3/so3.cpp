#include "se3.hpp"

SO3::SO3(Eigen::Quaterniond const& quaternion) : mQuaternion(quaternion) {
}
