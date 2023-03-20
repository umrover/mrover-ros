#pragma once

#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

using R3 = Eigen::Vector3d;
using SO3 = Eigen::Quaterniond;

class SE3 {
private:
    static SE3 fromTf(geometry_msgs::Transform const& transform);

    static SE3 fromPose(geometry_msgs::Pose const& pose);

    R3 position;
    SO3 rotation;

    [[nodiscard]] geometry_msgs::Pose toPose() const;

    [[nodiscard]] geometry_msgs::Transform toTransform() const;

    [[nodiscard]] geometry_msgs::PoseStamped toPoseStamped(std::string const& frameId) const;

    [[nodiscard]] geometry_msgs::TransformStamped toTransformStamped(const std::string& parentFrameId, const std::string& childFrameId) const;

public:
    [[nodiscard]] static SE3 fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId);

    static void pushToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3 const& tf);

    SE3() = default;

    explicit SE3(R3 position, SO3 const& rotation = SO3::Identity());

    [[nodiscard]] SE3 applyLeft(SE3 const& transform);

    [[nodiscard]] SE3 applyRight(SE3 const& transform);

    [[nodiscard]] R3 const& positionVector() const;

    [[nodiscard]] SO3 const& rotationQuaternion() const;

    [[nodiscard]] Eigen::Matrix4d rotationMatrix() const;

    [[nodiscard]] double distanceTo(SE3 const& other);
};
