#include "se3.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/**
 * TODO: document
 * @param buffer
 * @param fromFrameId
 * @param toFrameId
 * @param from
 * @param seq
 * @return
 */
SE3 SE3::fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId, SE3 from, uint32_t seq) {
    geometry_msgs::PoseStamped fromPose{};
    fromPose.header.frame_id = fromFrameId;
    fromPose.header.stamp = ros::Time::now();
    fromPose.header.seq = seq;
    fromPose.pose = static_cast<geometry_msgs::Pose>(from);

    geometry_msgs::PoseStamped fromInToPose{};
    geometry_msgs::TransformStamped transform = buffer.lookupTransform(toFrameId, fromFrameId, ros::Time(0));
    tf2::doTransform(fromPose, fromInToPose, transform);

    SE3 fromInTo{};
    fromInTo.position = fromInToPose.pose.position;
    fromInTo.orientation = fromInToPose.pose.orientation;
    return fromInTo;
}

/**
 * TODO: document
 * @param broadcaster
 * @param childFrameId
 * @param parentFrameId
 * @param send
 * @param seq
 */
void SE3::sendTfToTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3 send, uint32_t seq) {
    geometry_msgs::TransformStamped tf{};
    tf.child_frame_id = childFrameId;
    tf.header.frame_id = parentFrameId;
    tf.header.stamp = ros::Time::now();
    tf.header.seq = seq;
    tf.transform.translation.x = send.position.x;
    tf.transform.translation.y = send.position.y;
    tf.transform.translation.z = send.position.z;
    tf.transform.rotation = send.orientation;
    broadcaster.sendTransform(tf);
}
