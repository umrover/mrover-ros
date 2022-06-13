#include "se3.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

SE3 SE3::transform(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId, SE3 from, uint32_t seq) {
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