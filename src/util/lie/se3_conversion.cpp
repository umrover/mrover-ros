#include "se3.hpp"

SE3d SE3Conversions::fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId) {
    geometry_msgs::TransformStamped transform = buffer.lookupTransform(fromFrameId, toFrameId, ros::Time{});
    return fromTf(transform.transform);
}

void SE3Conversions::pushToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3d const& tf) {
    broadcaster.sendTransform(toTransformStamped(tf, parentFrameId, childFrameId));
}

SE3d SE3Conversions::fromTf(geometry_msgs::Transform const& transform) {
    return {{transform.translation.x, transform.translation.y, transform.translation.z},
            Eigen::Quaterniond{transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z}};
}

SE3d SE3Conversions::fromPose(geometry_msgs::Pose const& pose) {
    return {{pose.position.x, pose.position.y, pose.position.z},
            Eigen::Quaterniond{pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z}};
}

geometry_msgs::Pose SE3Conversions::toPose(SE3d const& tf) {
    geometry_msgs::Pose pose;
    pose.position.x = tf.x();
    pose.position.y = tf.y();
    pose.position.z = tf.z();
    pose.orientation.x = tf.quat().x();
    pose.orientation.y = tf.quat().y();
    pose.orientation.z = tf.quat().z();
    pose.orientation.w = tf.quat().w();
    return pose;
}

geometry_msgs::Transform SE3Conversions::toTransform(SE3d const& tf) {
    geometry_msgs::Transform transform;
    transform.translation.x = tf.x();
    transform.translation.y = tf.y();
    transform.translation.z = tf.z();
    transform.rotation.x = tf.quat().x();
    transform.rotation.y = tf.quat().y();
    transform.rotation.z = tf.quat().z();
    transform.rotation.w = tf.quat().w();
    return transform;
}

geometry_msgs::PoseStamped SE3Conversions::toPoseStamped(SE3d const& tf, std::string const& frameId) {
    geometry_msgs::PoseStamped pose;
    pose.pose = toPose(tf);
    pose.header.frame_id = frameId;
    pose.header.stamp = ros::Time::now();
    return pose;
}

geometry_msgs::TransformStamped SE3Conversions::toTransformStamped(SE3d const& tf, std::string const& parentFrameId, std::string const& childFrameId) {
    geometry_msgs::TransformStamped transform;
    transform.transform = toTransform(tf);
    transform.header.frame_id = parentFrameId;
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = childFrameId;
    return transform;
}

SE3 SE3::fromTf(geometry_msgs::Transform const& transform) {
    return {{transform.translation.x, transform.translation.y, transform.translation.z},
            {Eigen::Quaterniond{transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z}}};
}

SE3 SE3::fromPose(geometry_msgs::Pose const& pose) {
    return {{pose.position.x, pose.position.y, pose.position.z},
            {Eigen::Quaterniond{pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z}}};
}

geometry_msgs::Pose SE3::toPose() const {
    geometry_msgs::Pose pose;
    pose.position.x = position().x();
    pose.position.y = position().y();
    pose.position.z = position().z();
    pose.orientation.x = rotation().quaternion().x();
    pose.orientation.y = rotation().quaternion().y();
    pose.orientation.z = rotation().quaternion().z();
    pose.orientation.w = rotation().quaternion().w();
    return pose;
}

geometry_msgs::Transform SE3::toTransform() const {
    geometry_msgs::Transform transform;
    transform.translation.x = position().x();
    transform.translation.y = position().y();
    transform.translation.z = position().z();
    transform.rotation.x = rotation().quaternion().x();
    transform.rotation.y = rotation().quaternion().y();
    transform.rotation.z = rotation().quaternion().z();
    transform.rotation.w = rotation().quaternion().w();
    return transform;
}

geometry_msgs::PoseStamped SE3::toPoseStamped(std::string const& frameId) const {
    geometry_msgs::PoseStamped pose;
    pose.pose = toPose();
    pose.header.frame_id = frameId;
    pose.header.stamp = ros::Time::now();
    return pose;
}

geometry_msgs::TransformStamped SE3::toTransformStamped(std::string const& parentFrameId, std::string const& childFrameId) const {
    geometry_msgs::TransformStamped transform;
    transform.transform = toTransform();
    transform.header.frame_id = parentFrameId;
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = childFrameId;
    return transform;
}
