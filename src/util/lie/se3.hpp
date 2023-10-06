#pragma once

#include <boost_cpp23_workaround.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Geometry>

using R3 = Eigen::Vector3d;

/**
 * @brief A 3D rotation.
 */
class SO3 {
private:
    using AngleAxis = Eigen::AngleAxis<double>;

    AngleAxis mAngleAxis = AngleAxis::Identity();

public:
    friend class SE3;

    // enable_if_t ensures if we add other explicit constructors this one fails quickly
    template<typename... Args, typename = std::enable_if_t<std::is_constructible_v<AngleAxis, Args...>>>
    SO3(Args&&... args) : mAngleAxis{std::forward<Args>(args)...} {
    }

    [[nodiscard]] SO3 operator*(SO3 const& other) const;

    [[nodiscard]] R3 operator*(R3 const& other) const;

    [[nodiscard]] Eigen::Matrix4d matrix() const;

    [[nodiscard]] Eigen::Quaterniond quaternion() const;
};

/**
 * @brief A 3D rigid transformation (direct isometry).
 *
 * In simpler terms: a 3D rotation followed by a 3D translation.
 */
class SE3 {
private:
    using Transform = Eigen::Transform<double, 3, Eigen::Isometry>;

    static SE3 fromTf(geometry_msgs::Transform const& transform);

    static SE3 fromPose(geometry_msgs::Pose const& pose);

    Transform mTransform = Transform::Identity();

    [[nodiscard]] geometry_msgs::Pose toPose() const;

    [[nodiscard]] geometry_msgs::Transform toTransform() const;

    [[nodiscard]] geometry_msgs::PoseStamped toPoseStamped(std::string const& frameId) const;

    [[nodiscard]] geometry_msgs::TransformStamped toTransformStamped(const std::string& parentFrameId, const std::string& childFrameId) const;

public:
    [[nodiscard]] static SE3 fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId);

    static void pushToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3 const& tf);

    SE3(R3 const& position, SO3 const& rotation = {});

    template<typename... Args, typename = std::enable_if_t<std::is_constructible_v<Transform, Args...>>>
    SE3(Args&&... args) : mTransform{std::forward<Args>(args)...} {
    }

    [[nodiscard]] SE3 operator*(SE3 const& other) const;

    [[nodiscard]] Eigen::Matrix4d matrix() const;

    [[nodiscard]] R3 position() const;

    [[nodiscard]] SO3 rotation() const;

    [[nodiscard]] double distanceTo(SE3 const& other) const;
};
