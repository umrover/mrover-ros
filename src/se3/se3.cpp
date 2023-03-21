#include "se3.hpp"

#include <utility>

/**
 *
 * @param position
 * @param rotation
 */
SE3::SE3(Eigen::Vector3d position, SO3 const& rotation) : mPosition(std::move(position)), mRotation(rotation) {
}

/**
 *
 * @param transform
 * @return
 */
SE3 SE3::applyLeft(SE3 const& transform) {
    (void) this;
    (void) transform;
    throw std::logic_error("Not implemented yet!");
}

/**
 *
 * @param transform
 * @return
 */
SE3 SE3::applyRight(SE3 const& transform) {
    (void) this;
    (void) transform;
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
R3 const& SE3::positionVector() const {
    return mPosition;
}

/**
 *
 * @return
 */
SO3 const& SE3::rotation() const {
    return mRotation;
}

/**
 *
 * @param other
 * @return
 */
double SE3::distanceTo(SE3 const& other) {
    return (mPosition - other.mPosition).squaredNorm();
}
