#include <format>
#include <numbers>
#include <numeric>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <boost/circular_buffer.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

#include <lie.hpp>

static ros::Duration const STEP{0.5}, WINDOW{2 + STEP.toSec() / 2};

constexpr static float MIN_LINEAR_SPEED = 0.2, MAX_ANGULAR_SPEED = 0.1, MAX_ANGULAR_CHANGE = 0.2;

auto squared(auto const& x) { return x * x; }

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "heading_correcting_filter");
    ros::NodeHandle nh;

    auto useOdomFrame = nh.param<bool>("use_odom", false);
    auto roverFrame = nh.param<std::string>("rover_frame", "base_link");
    auto mapFrame = nh.param<std::string>("map_frame", "map");

    if (useOdomFrame) throw std::runtime_error{"Not supported"};

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};

    tf2_ros::TransformBroadcaster tfBroadcaster;

    geometry_msgs::Twist currentTwist;

    SO3d uncorrectedOrientation = SO3d::Identity(), correctionRotation = SO3d::Identity();

    ros::Timer correctTimer = nh.createTimer(WINDOW, [&](ros::TimerEvent const&) {
        // 1. Ensure the rover is being commanded to move relatively straight forward
        if (std::fabs(currentTwist.linear.x) < MIN_LINEAR_SPEED) return;
        if (std::fabs(currentTwist.angular.z) > MAX_ANGULAR_SPEED) return;

        ROS_INFO("Rover is being commanded forward");

        // Compute the past velocities and headings of the rover in the map frame over a window of time

        R2 roverVelocitySum{};
        double roverHeadingSum = 0.0;
        std::size_t readings = 0;

        ros::Time end = ros::Time::now(), start = end - WINDOW;
        for (ros::Time t = start; t < end; t += STEP) {
            try {
                auto roverInMapOld = SE3Conversions::fromTfTree(tfBuffer, roverFrame, mapFrame, t - STEP);
                auto roverInMapNew = SE3Conversions::fromTfTree(tfBuffer, roverFrame, mapFrame, t);
                R3 roverVelocityInMap = (roverInMapNew.translation() - roverInMapOld.translation()) / STEP.toSec();
                R3 roverAngularVelocityInMap = (roverInMapNew.asSO3() - roverInMapOld.asSO3()).coeffs();
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

        if (R2 meanVelocityInMap = roverVelocitySum / static_cast<double>(readings); meanVelocityInMap.norm() < MIN_LINEAR_SPEED) {
            ROS_INFO_STREAM(std::format("Rover is not moving fast enough: speed = {} m/s", meanVelocityInMap.norm()));
            return;
        }

        if (roverHeadingSum > MAX_ANGULAR_CHANGE) {
            ROS_INFO_STREAM(std::format("Rover is not moving straight enough: heading change = {} rad", roverHeadingSum));
            return;
        }

        double correctedHeadingInMap = std::atan2(roverVelocitySum.y(), roverVelocitySum.x());

        R2 uncorrectedForward = uncorrectedOrientation.rotation().col(0).head<2>();
        double estimatedHeadingInMap = std::atan2(uncorrectedForward.y(), uncorrectedForward.x());

        double headingCorrectionDelta = correctedHeadingInMap - estimatedHeadingInMap;

        correctionRotation = Eigen::AngleAxisd{headingCorrectionDelta, R3::UnitZ()};

        ROS_INFO_STREAM(std::format("Correcting heading by: {}", correctionRotation.z()));
    });

    ros::Subscriber twistSubscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, [&](geometry_msgs::TwistConstPtr const& twist) {
        currentTwist = *twist;
    });

    ros::Subscriber poseSubscriber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/linearized_pose", 1, [&](geometry_msgs::PoseWithCovarianceStampedConstPtr const& pose) {
        geometry_msgs::Quaternion const& o = pose->pose.pose.orientation;
        geometry_msgs::Point const& p = pose->pose.pose.position;
        SE3d uncorrectedPose{R3{p.x, p.y, p.z}, S3{o.w, o.x, o.y, o.z}};
        uncorrectedOrientation = uncorrectedPose.asSO3();

        SE3d correctedPose{uncorrectedPose.translation(), correctionRotation * uncorrectedOrientation};
        SE3Conversions::pushToTfTree(tfBroadcaster, roverFrame, mapFrame, correctedPose);
    });

    ros::spin();

    return EXIT_SUCCESS;
}