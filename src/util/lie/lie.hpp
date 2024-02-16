#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Geometry>
#include <manif/SE3.h>

using manif::SE3d, manif::SO3d;

using R3 = Eigen::Vector3d;
using S3 = Eigen::Quaterniond;

class SE3Conversions {
public:
    static auto fromTf(geometry_msgs::Transform const& transform) -> SE3d;

    static auto fromPose(geometry_msgs::Pose const& pose) -> SE3d;

    [[nodiscard]] static auto toPose(SE3d const& tf) -> geometry_msgs::Pose;

    [[nodiscard]] static auto toTransform(SE3d const& tf) -> geometry_msgs::Transform;

    [[nodiscard]] static auto toPoseStamped(SE3d const& tf, std::string const& frameId) -> geometry_msgs::PoseStamped;

    [[nodiscard]] static auto toTransformStamped(SE3d const& tf, std::string const& parentFrameId, std::string const& childFrameId) -> geometry_msgs::TransformStamped;

    [[nodiscard]] static auto fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId) -> SE3d;

    static auto pushToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3d const& tf) -> void;
};

class SIM3 {
    using Transform = Eigen::Transform<double, 3, Eigen::Affine>;

    Transform mTransform = Transform::Identity();

public:
    SIM3() = default;

    SIM3(SE3d const& se3, R3 const& scale);

    [[nodiscard]] auto matrix() const -> Eigen::Matrix4d;

    [[nodiscard]] auto position() const -> R3;
};
