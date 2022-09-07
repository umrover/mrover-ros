#include "se3.hpp"

SE3 SE3::fromTf(geometry_msgs::Transform const& transform) {
    SE3 se3;
    se3.mPosition.x() = transform.translation.x;
    se3.mPosition.y() = transform.translation.y;
    se3.mPosition.z() = transform.translation.z;
    se3.mRotation.mQuaternion.x() = transform.rotation.x;
    se3.mRotation.mQuaternion.y() = transform.rotation.y;
    se3.mRotation.mQuaternion.z() = transform.rotation.z;
    se3.mRotation.mQuaternion.w() = transform.rotation.w;
    return se3;
}

SE3 SE3::fromPose(geometry_msgs::Pose const& pose) {
    SE3 se3;
    se3.mPosition.x() = pose.position.x;
    se3.mPosition.y() = pose.position.y;
    se3.mPosition.z() = pose.position.z;
    se3.mRotation.mQuaternion.x() = pose.orientation.x;
    se3.mRotation.mQuaternion.y() = pose.orientation.y;
    se3.mRotation.mQuaternion.z() = pose.orientation.z;
    se3.mRotation.mQuaternion.w() = pose.orientation.w;
    return se3;
}

geometry_msgs::Pose SE3::toPose() const {
    geometry_msgs::Pose pose;
    pose.position.x = mPosition.x();
    pose.position.y = mPosition.y();
    pose.position.z = mPosition.z();
    pose.orientation.x = mRotation.mQuaternion.x();
    pose.orientation.y = mRotation.mQuaternion.y();
    pose.orientation.z = mRotation.mQuaternion.z();
    pose.orientation.w = mRotation.mQuaternion.w();
    return pose;
}

geometry_msgs::Transform SE3::toTransform() const {
    geometry_msgs::Transform transform;
    transform.translation.x = mPosition.x();
    transform.translation.y = mPosition.y();
    transform.translation.z = mPosition.z();
    transform.rotation.x = mRotation.mQuaternion.x();
    transform.rotation.y = mRotation.mQuaternion.y();
    transform.rotation.z = mRotation.mQuaternion.z();
    transform.rotation.w = mRotation.mQuaternion.w();
    return transform;
}

geometry_msgs::PoseStamped SE3::toPoseStamped(std::string const& frame) const {
    geometry_msgs::PoseStamped pose;
    pose.pose = toPose();
    pose.header.frame_id = frame;
    pose.header.stamp = ros::Time::now();
    return pose;
}

geometry_msgs::TransformStamped SE3::toTransformStamped(const std::string& parentFrame, const std::string& childFrame) const {
    geometry_msgs::TransformStamped transform;
    transform.transform = toTransform();
    transform.header.frame_id = parentFrame;
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = childFrame;
    return transform;
}
