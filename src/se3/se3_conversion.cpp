#include "se3.hpp"

SE3 SE3::fromTf(geometry_msgs::Transform const& transform) {
    SE3 se3;
    se3.position.x() = transform.translation.x;
    se3.position.y() = transform.translation.y;
    se3.position.z() = transform.translation.z;
    se3.rotation.x() = transform.rotation.x;
    se3.rotation.y() = transform.rotation.y;
    se3.rotation.z() = transform.rotation.z;
    se3.rotation.w() = transform.rotation.w;
    return se3;
}

SE3 SE3::fromPose(geometry_msgs::Pose const& pose) {
    SE3 se3;
    se3.position.x() = pose.position.x;
    se3.position.y() = pose.position.y;
    se3.position.z() = pose.position.z;
    se3.rotation.x() = pose.orientation.x;
    se3.rotation.y() = pose.orientation.y;
    se3.rotation.z() = pose.orientation.z;
    se3.rotation.w() = pose.orientation.w;
    return se3;
}

geometry_msgs::Pose SE3::toPose() const {
    geometry_msgs::Pose pose;
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();
    pose.orientation.x = rotation.x();
    pose.orientation.y = rotation.y();
    pose.orientation.z = rotation.z();
    pose.orientation.w = rotation.w();
    return pose;
}

geometry_msgs::Transform SE3::toTransform() const {
    geometry_msgs::Transform transform;
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();
    transform.rotation.x = rotation.x();
    transform.rotation.y = rotation.y();
    transform.rotation.z = rotation.z();
    transform.rotation.w = rotation.w();
    return transform;
}

geometry_msgs::PoseStamped SE3::toPoseStamped(std::string const& frameId) const {
    geometry_msgs::PoseStamped pose;
    pose.pose = toPose();
    pose.header.frame_id = frameId;
    pose.header.stamp = ros::Time::now();
    return pose;
}

geometry_msgs::TransformStamped SE3::toTransformStamped(const std::string& parentFrameId, const std::string& childFrameId) const {
    geometry_msgs::TransformStamped transform;
    transform.transform = toTransform();
    transform.header.frame_id = parentFrameId;
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = childFrameId;
    return transform;
}
