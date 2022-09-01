#include "se3.hpp"

SE3 SE3::fromTf(geometry_msgs::Transform const& transform) {
    SE3 se3;
    se3.position.x() = transform.translation.x;
    se3.position.y() = transform.translation.y;
    se3.position.z() = transform.translation.z;
    se3.rotation.quaternion.x() = transform.rotation.x;
    se3.rotation.quaternion.y() = transform.rotation.y;
    se3.rotation.quaternion.z() = transform.rotation.z;
    se3.rotation.quaternion.w() = transform.rotation.w;
    return se3;
}

SE3 SE3::fromPose(geometry_msgs::Pose const& pose) {
    SE3 se3;
    se3.position.x() = pose.position.x;
    se3.position.y() = pose.position.y;
    se3.position.z() = pose.position.z;
    se3.rotation.quaternion.x() = pose.orientation.x;
    se3.rotation.quaternion.y() = pose.orientation.y;
    se3.rotation.quaternion.z() = pose.orientation.z;
    se3.rotation.quaternion.w() = pose.orientation.w;
    return se3;
}

geometry_msgs::Pose SE3::toPose() const {
    geometry_msgs::Pose pose;
    pose.position.x = position.x();
    pose.position.y = position.y();
    pose.position.z = position.z();
    pose.orientation.x = rotation.quaternion.x();
    pose.orientation.y = rotation.quaternion.y();
    pose.orientation.z = rotation.quaternion.z();
    pose.orientation.w = rotation.quaternion.w();
    return pose;
}

geometry_msgs::Transform SE3::toTransform() const {
    geometry_msgs::Transform transform;
    transform.translation.x = position.x();
    transform.translation.y = position.y();
    transform.translation.z = position.z();
    transform.rotation.x = rotation.quaternion.x();
    transform.rotation.y = rotation.quaternion.y();
    transform.rotation.z = rotation.quaternion.z();
    transform.rotation.w = rotation.quaternion.w();
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
