#pragma once

#include <string>

#include <tf2_ros/buffer.h>

#include <geometry_msgs/Pose.h>

class SE3 : public geometry_msgs::Pose {
public:
    static SE3 transform(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId, SE3 from, uint32_t seq);
};
