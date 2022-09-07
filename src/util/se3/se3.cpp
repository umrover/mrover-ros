#include "se3.hpp"

#include <utility>

SE3::SE3(Vector position, SO3 rotation) : mPosition(std::move(position)), mRotation(std::move(rotation)) {
}

SE3 SE3::fromPosQuat(Vector const& position, Quaternion const& rotation) {
    return SE3{position, SO3{rotation}};
}

SE3 SE3::applyLeft(SE3 const& other) {
    auto affine = Eigen::Affine3d::Identity();
    affine.translate(other.mPosition);
    affine.rotate(other.mRotation.mQuaternion);
    return {affine * mPosition, SO3{other.mRotation.mQuaternion * mRotation.mQuaternion}};
}

SE3 SE3::applyRight([[maybe_unused]] SE3 const& other) {
    throw std::logic_error("Not implemented yet!");
}

SE3 SE3::fromTfTree(tf2_ros::Buffer const& buffer, std::string const& parentFrame, std::string const& childFrame) {
    geometry_msgs::TransformStamped transform = buffer.lookupTransform(parentFrame, childFrame, ros::Time(0));
    return SE3::fromTf(transform.transform);
}

void SE3::publishToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrame, std::string const& parentFrame) const {
    broadcaster.sendTransform(toTransformStamped(parentFrame, childFrame));
}

double SE3::posDistanceTo(SE3 const& other) const {
    return (mPosition - other.mPosition).squaredNorm();
}

SE3::Vector const& SE3::posVector() const {
    return mPosition;
}

SO3 const& SE3::rotation() const {
    return mRotation;
}

bool SE3::isApprox(SE3 const& other, double tolerance) const {
    return mPosition.isApprox(other.mPosition, tolerance) && mRotation.isApprox(other.mRotation, tolerance);
}
