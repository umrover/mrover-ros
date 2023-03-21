#include "se3.hpp"

#include <utility>

SE3::SE3(Eigen::Vector3d position, SO3 rotation) : mPosition(std::move(position)), mRotation(std::move(rotation)) {
}

SE3 SE3::fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId) {
    geometry_msgs::TransformStamped transform = buffer.lookupTransform(fromFrameId, toFrameId, ros::Time(0));
    return SE3::fromTf(transform.transform);
}

void SE3::pushToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3 const& tf) {
    broadcaster.sendTransform(tf.toTransformStamped(parentFrameId, childFrameId));
}

Eigen::Matrix4d SE3::matrix() const {
    auto affine = Eigen::Affine3d::Identity();
    affine.rotate(mRotation.mQuaternion);
    affine.translate(mPosition);
    return affine.matrix();
}

R3 const& SE3::position() const {
    return mPosition;
}

SO3 const& SE3::rotation() const {
    return mRotation;
}

double SE3::distanceTo(SE3 const& other) {
    return (mPosition - other.mPosition).squaredNorm();
}
