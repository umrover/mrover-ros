#include <format>
#include <numeric>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <boost/circular_buffer.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <lie.hpp>

static ros::Duration const STEP{0.5}, WINDOW{2 + STEP.toSec() / 2};

constexpr static float MIN_LINEAR_SPEED = 0.2, MAX_ANGULAR_SPEED = 0.1, MAX_ANGULAR_CHANGE = 0.2;

auto rosQuaternionToEigenQuaternion(geometry_msgs::Quaternion const& q) -> Eigen::Quaterniond {
    return {q.w, q.x, q.y, q.z};
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "pose_filter");
    ros::NodeHandle nh;

    auto roverFrame = nh.param<std::string>("rover_frame", "base_link");
    auto mapFrame = nh.param<std::string>("map_frame", "map");

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};
    tf2_ros::TransformBroadcaster tfBroadcaster;

    ros::Publisher odometryPub = nh.advertise<nav_msgs::Odometry>("/odometry", 1);

    std::optional<geometry_msgs::Twist> currentTwist;
    std::optional<sensor_msgs::Imu> currentImuCalib, currentImuUncalib;

    std::optional<SO3d> correctionRotation;

    std::optional<SE3d> lastPose;
    std::optional<ros::Time> lastPoseTime;

    ros::Timer correctTimer = nh.createTimer(WINDOW, [&](ros::TimerEvent const&) {
        if (!currentTwist || !currentImuUncalib) return;

        // 1. Ensure the rover is being commanded to move relatively straight forward
        if (currentTwist->linear.x < MIN_LINEAR_SPEED) return;
        if (std::fabs(currentTwist->angular.z) > MAX_ANGULAR_SPEED) return;

        ROS_INFO("Rover is being commanded forward");

        // Compute the past velocities and headings of the rover in the map frame over a window of time

        R2d roverVelocitySum{};
        double roverHeadingSum = 0.0;
        std::size_t readings = 0;

        ros::Time end = ros::Time::now(), start = end - WINDOW;
        for (ros::Time t = start; t < end; t += STEP) {
            try {
                auto roverInMapOld = SE3Conversions::fromTfTree(tfBuffer, roverFrame, mapFrame, t - STEP);
                auto roverInMapNew = SE3Conversions::fromTfTree(tfBuffer, roverFrame, mapFrame, t);
                R3d roverVelocityInMap = (roverInMapNew.translation() - roverInMapOld.translation()) / STEP.toSec();
                R3d roverAngularVelocityInMap = (roverInMapNew.asSO3() - roverInMapOld.asSO3()).coeffs();
                roverVelocitySum += roverVelocityInMap.head<2>();
                roverHeadingSum += roverAngularVelocityInMap.z();
                ++readings;
            } catch (tf2::ConnectivityException const& e) {
                ROS_WARN_STREAM(e.what());
                return;
            } catch (tf2::LookupException const& e) {
                ROS_WARN_STREAM(e.what());
                return;
            } catch (tf2::ExtrapolationException const&) {
            }
        }

        // 2. Ensure the rover has actually moved to avoid correcting while standing still

        if (R2d meanVelocityInMap = roverVelocitySum / static_cast<double>(readings); meanVelocityInMap.norm() < MIN_LINEAR_SPEED) {
            ROS_INFO_STREAM(std::format("Rover is not moving fast enough: speed = {} m/s", meanVelocityInMap.norm()));
            return;
        }

        if (roverHeadingSum > MAX_ANGULAR_CHANGE) {
            ROS_INFO_STREAM(std::format("Rover is not moving straight enough: heading change = {} rad", roverHeadingSum));
            return;
        }

        double correctedHeadingInMap = std::atan2(roverVelocitySum.y(), roverVelocitySum.x());

        SO3d uncorrectedOrientation = rosQuaternionToEigenQuaternion(currentImuUncalib->orientation);
        R2d uncorrectedForward = uncorrectedOrientation.rotation().col(0).head<2>();
        double estimatedHeadingInMap = std::atan2(uncorrectedForward.y(), uncorrectedForward.x());

        double headingCorrectionDelta = correctedHeadingInMap - estimatedHeadingInMap;

        correctionRotation = Eigen::AngleAxisd{headingCorrectionDelta, R3d::UnitZ()};

        ROS_INFO_STREAM(std::format("Correcting heading by: {}", correctionRotation->z()));
    });

    ros::Subscriber twistSubscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, [&](geometry_msgs::TwistConstPtr const& twist) {
        currentTwist = *twist;
    });

    ros::Subscriber imuCalibSubscriber = nh.subscribe<sensor_msgs::Imu>("/imu/data", 1, [&](sensor_msgs::ImuConstPtr const& imu) {
        currentImuCalib = *imu;
    });

    ros::Subscriber imuUncalibSubscriber = nh.subscribe<sensor_msgs::Imu>("/imu/data_raw", 1, [&](sensor_msgs::ImuConstPtr const& imu) {
        currentImuUncalib = *imu;
    });

    ros::Subscriber poseSubscriber = nh.subscribe<geometry_msgs::Vector3Stamped>("/linearized_position", 1, [&](geometry_msgs::Vector3StampedConstPtr const& msg) {
        R3d position{msg->vector.x, msg->vector.y, msg->vector.z};

        SE3d pose{position, SO3d::Identity()};
        if (correctionRotation && currentImuUncalib) {
            SO3d uncalibratedOrientation = rosQuaternionToEigenQuaternion(currentImuUncalib->orientation);
            pose.asSO3() = correctionRotation.value() * uncalibratedOrientation;
        } else if (currentImuCalib) {
            SO3d calibratedOrientation = rosQuaternionToEigenQuaternion(currentImuCalib->orientation);
            pose.asSO3() = calibratedOrientation;
        } else {
            ROS_WARN_THROTTLE(1, "No IMU data available");
            return;
        }
        SE3Conversions::pushToTfTree(tfBroadcaster, roverFrame, mapFrame, pose);

        SE3d::Tangent twist;
        if (lastPose && lastPoseTime) {
            twist = (pose - lastPose.value()) / (msg->header.stamp - lastPoseTime.value()).toSec();
        }

        nav_msgs::Odometry odometry;
        odometry.header = msg->header;
        odometry.pose.pose.position.x = pose.translation().x();
        odometry.pose.pose.position.y = pose.translation().y();
        odometry.pose.pose.position.z = pose.translation().z();
        odometry.pose.pose.orientation.w = pose.quat().w();
        odometry.pose.pose.orientation.x = pose.quat().x();
        odometry.pose.pose.orientation.y = pose.quat().y();
        odometry.pose.pose.orientation.z = pose.quat().z();
        odometry.twist.twist.linear.x = twist.coeffs()(0);
        odometry.twist.twist.linear.y = twist.coeffs()(1);
        odometry.twist.twist.linear.z = twist.coeffs()(2);
        odometry.twist.twist.angular.x = twist.coeffs()(3);
        odometry.twist.twist.angular.y = twist.coeffs()(4);
        odometry.twist.twist.angular.z = twist.coeffs()(5);
        odometryPub.publish(odometry);

        lastPose = pose;
        lastPoseTime = msg->header.stamp;
    });

    ros::spin();

    return EXIT_SUCCESS;
}