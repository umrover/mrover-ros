#include "se3_ros_utils.hpp"

namespace mrover {
    SE3d SE3fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId) {
        geometry_msgs::TransformStamped transform = buffer.lookupTransform(fromFrameId, toFrameId, ros::Time{});
        return SE3fromTf(transform.transform);
    }

    void pushSE3ToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3d const& tf) {
        broadcaster.sendTransform(SE3toTransformStamped(tf, parentFrameId, childFrameId));
    }

    SE3d SE3fromTf(geometry_msgs::Transform const& transform) {
        return {{transform.translation.x, transform.translation.y, transform.translation.z},
                Eigen::Quaterniond{transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z}};
    }

    SE3d SE3fromPose(geometry_msgs::Pose const& pose) {
        return {{pose.position.x, pose.position.y, pose.position.z},
                Eigen::Quaterniond{pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z}};
    }

    geometry_msgs::Pose SE3toPose(SE3d const& tf) {
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

    geometry_msgs::Transform SE3toTransform(SE3d const& tf) {
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

    geometry_msgs::PoseStamped SE3toPoseStamped(SE3d const& tf, std::string const& frameId) {
        geometry_msgs::PoseStamped pose;
        pose.pose = SE3toPose(tf);
        pose.header.frame_id = frameId;
        pose.header.stamp = ros::Time::now();
        return pose;
    }

    geometry_msgs::TransformStamped SE3toTransformStamped(SE3d const& tf, std::string const& parentFrameId, std::string const& childFrameId) {
        geometry_msgs::TransformStamped transform;
        transform.transform = SE3toTransform(tf);
        transform.header.frame_id = parentFrameId;
        transform.header.stamp = ros::Time::now();
        transform.child_frame_id = childFrameId;
        return transform;
    }
} // namespace mrover