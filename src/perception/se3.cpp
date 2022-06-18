#include "se3.hpp"

/**
 *
 * @param position
 * @param rotation
 */
SE3::SE3(Eigen::Vector3d const& position, Eigen::Quaterniond const& rotation) : position(position), rotation(rotation) {
}

/**
 *
 * @param pose
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
