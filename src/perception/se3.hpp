#pragma once

#include "so3.hpp"

#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

class SE3 {
private:
    using Quaternion = Eigen::Quaterniond;
    using Vector = Eigen::Vector3d;

    static SE3 fromTf(geometry_msgs::Transform const& transform);

    static SE3 fromPose(geometry_msgs::Pose const& pose);

    Vector position;
    SO3 rotation;

    [[nodiscard]] geometry_msgs::Pose toPose() const;

    [[nodiscard]] geometry_msgs::Transform toTransform() const;

    [[nodiscard]] geometry_msgs::PoseStamped toPoseStamped(std::string const& frame) const;

    [[nodiscard]] geometry_msgs::TransformStamped toTransformStamped(const std::string& parentFrame, const std::string& childFrame) const;

public:
    [[nodiscard]] static SE3 fromPosQuat(Vector const& position, Quaternion const& rotation);

    [[nodiscard]] static SE3 fromTfTree(tf2_ros::Buffer const& buffer, std::string const& parentFrame, std::string const& childFrame);

    SE3() = default;

    SE3(Vector position, SO3 rotation);

    void publishToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrame, std::string const& parentFrame);

    [[nodiscard]] SE3 applyLeft(SE3 const& other);

    [[nodiscard]] SE3 applyRight(SE3 const& other);

    [[nodiscard]] double posDistanceTo(SE3 const& other);

    [[nodiscard]] Vector const& posVector() const;
};
