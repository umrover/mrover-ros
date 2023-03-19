#include "se3.hpp"

#include <utility>

/**
 *
 * @param position
 * @param rotation
 */
SE3::SE3(Eigen::Vector3d position, Eigen::Quaterniond const& rotation) : position(std::move(position)), rotation(rotation) {
}

/**
 *
 * @param transform
 * @return
 */
SE3 SE3::applyLeft(SE3 const& transform) {
    auto affine = Eigen::Affine3d::Identity();
    affine.translate(transform.position);
    affine.rotate(transform.rotation);
    return {affine * position, transform.rotation * rotation};
}

/**
 *
 * @param transform
 * @return
 */
SE3 SE3::applyRight([[maybe_unused]] SE3 const& transform) {
    (void) this;
    throw std::logic_error("Not implemented yet!");
}

/**
 *
 * @param buffer
 * @param fromFrameId
 * @param toFrameId
 * @return
 */
SE3 SE3::fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId) {
    geometry_msgs::TransformStamped transform = buffer.lookupTransform(fromFrameId, toFrameId, ros::Time(0));
    return SE3::fromTf(transform.transform);
}

/**
 *
 * @param broadcaster
 * @param childFrameId
 * @param parentFrameId
 * @param tf
 */
void SE3::pushToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3 const& tf) {
    broadcaster.sendTransform(tf.toTransformStamped(parentFrameId, childFrameId));
}

/**
 *
 * @return
 */
Eigen::Vector3d const& SE3::positionVector() const {
    return position;
}

/**
 *
 * @return
 */
Eigen::Quaterniond const& SE3::rotationQuaternion() const {
    return rotation;
}

/**
 *
 * @return
 */
Eigen::Matrix4d SE3::rotationMatrix() const {
    auto affine = Eigen::Affine3d::Identity();
    affine.rotate(rotation);
    return affine.matrix();
}

/**
 *
 * @param other
 * @return
 */
double SE3::distanceTo(SE3 const& other) {
    return (position - other.position).squaredNorm();
}
