#pragma once

#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/Pose.h>

class SE3 : public geometry_msgs::Pose {
public:
    [[nodiscard]] static SE3 transform(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId, SE3 from, uint32_t seq);

    static void sendTransform(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3 send, uint32_t seq);
};
