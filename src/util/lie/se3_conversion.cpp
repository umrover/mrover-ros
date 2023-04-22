#include "se3.hpp"

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

geometry_msgs::TransformStamped SE3::toTransformStamped(const std::string& parentFrameId, const std::string& childFrameId) const {
    geometry_msgs::TransformStamped transform;
    transform.transform = toTransform();
    transform.header.frame_id = parentFrameId;
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = childFrameId;
    return transform;
}
