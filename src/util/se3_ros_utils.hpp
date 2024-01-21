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


SE3d SE3fromTf(geometry_msgs::Transform const& transform);

SE3d SE3fromPose(geometry_msgs::Pose const& pose);

[[nodiscard]] geometry_msgs::Pose SE3toPose(SE3d const& tf);

[[nodiscard]] geometry_msgs::Transform SE3toTransform(SE3d const& tf);

[[nodiscard]] geometry_msgs::PoseStamped SE3toPoseStamped(SE3d const& tf, std::string const& frameId);

[[nodiscard]] geometry_msgs::TransformStamped SE3toTransformStamped(SE3d const& tf, std::string const& parentFrameId, std::string const& childFrameId);

[[nodiscard]] SE3d SE3fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId);

void pushSE3ToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3d const& tf);
