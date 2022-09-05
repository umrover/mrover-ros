#pragma once

#include <string>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <Eigen/Dense>

class SO3 {

public:
    using Quaternion = Eigen::Quaterniond;
    using Vector = Eigen::Vector3d;
    using RotationMatrix = Eigen::Matrix3d;

private:
    friend class SE3;

    Quaternion quaternion;

public:
    [[nodiscard]] static SO3 identity();

    [[nodiscard]] static SO3 fromMatrix(Eigen::Matrix3d const& rotationMatrix);

    SO3() = default;

    explicit SO3(Quaternion const& quaternion);

    [[nodiscard]] RotationMatrix rotationMatrix();

    [[nodiscard]] Vector directionVector();

    [[nodiscard]] double rotDistanceTo(SO3 const& other);

    [[nodiscard]] bool isApprox(SO3 const& other, double tolerance = 1e-8);
};
