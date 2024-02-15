#pragma once

#include <boost_cpp23_workaround.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Geometry>
#include <manif/SE3.h>

using manif::SE3d;

using R3 = Eigen::Vector3d;
using S3 = Eigen::Quaterniond;

/**
 * @brief A 3D rotation.
 */
class SO3 {
    using AngleAxis = Eigen::AngleAxis<double>;

    AngleAxis mAngleAxis = AngleAxis::Identity();

public:
    friend class SE3;
    friend class SIM3;

    SO3() = default;

    SO3(double w, double x, double y, double z) : mAngleAxis{Eigen::Quaterniond{w, x, y, z}} {}

    template<typename... Args,
             typename = std::enable_if_t<std::is_constructible_v<AngleAxis, Args...>>,
             typename = std::enable_if_t<(sizeof...(Args) > 0)>>
    SO3(Args&&... args) : mAngleAxis{std::forward<Args>(args)...} {}

    [[nodiscard]] SO3 operator*(SO3 const& other) const;

    [[nodiscard]] R3 operator*(R3 const& other) const;

    [[nodiscard]] Eigen::Matrix3d matrix() const;

    [[nodiscard]] Eigen::Quaterniond quaternion() const;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief A 3D rigid transformation (direct isometry).
 *
 * In simpler terms: a 3D rotation followed by a 3D translation.
 */
class SE3 {
    using Transform = Eigen::Transform<double, 3, Eigen::Isometry>;

    Transform mTransform = Transform::Identity();

    static SE3 fromTf(geometry_msgs::Transform const& transform);

    static SE3 fromPose(geometry_msgs::Pose const& pose);

    [[nodiscard]] geometry_msgs::Pose toPose() const;

    [[nodiscard]] geometry_msgs::Transform toTransform() const;

    [[nodiscard]] geometry_msgs::PoseStamped toPoseStamped(std::string const& frameId) const;

    [[nodiscard]] geometry_msgs::TransformStamped toTransformStamped(std::string const& parentFrameId, std::string const& childFrameId) const;

public:
    [[nodiscard]] static SE3 fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId);

    static void pushToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3 const& tf);

    SE3() = default;

    SE3(R3 const& position, SO3 const& rotation);

    template<typename... Args,
             typename = std::enable_if_t<std::is_constructible_v<Transform, Args...>>,
             typename = std::enable_if_t<(sizeof...(Args) > 0)>>
    SE3(Args&&... args) : mTransform{std::forward<Args>(args)...} {}

    [[nodiscard]] SE3 operator*(SE3 const& other) const;

    [[nodiscard]] Eigen::Matrix4d matrix() const;

    [[nodiscard]] R3 position() const;

    [[nodiscard]] SO3 rotation() const;

    [[nodiscard]] double distanceTo(SE3 const& other) const;

    [[nodiscard]] auto inverse() const -> SE3;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class SE3Conversions {
public:
    static SE3d fromTf(geometry_msgs::Transform const& transform);

    static SE3d fromPose(geometry_msgs::Pose const& pose);

    [[nodiscard]] static geometry_msgs::Pose toPose(SE3d const& tf);

    [[nodiscard]] static geometry_msgs::Transform toTransform(SE3d const& tf);

    [[nodiscard]] static geometry_msgs::PoseStamped toPoseStamped(SE3d const& tf, std::string const& frameId);

    [[nodiscard]] static geometry_msgs::TransformStamped toTransformStamped(SE3d const& tf, std::string const& parentFrameId, std::string const& childFrameId);

    [[nodiscard]] static SE3d fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId);

    static void pushToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3d const& tf);
};

class SIM3 {
    using Transform = Eigen::Transform<double, 3, Eigen::Affine>;

    Transform mTransform = Transform::Identity();

public:
    SIM3() = default;

    SIM3(R3 const& position, SO3 const& rotation, R3 const& scale);

    [[nodiscard]] auto matrix() const -> Eigen::Matrix4d;

    [[nodiscard]] auto position() const -> R3;
};
