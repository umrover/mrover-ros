#include <format>

#include <ros/init.h>
#include <ros/node_handle.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <boost/circular_buffer.hpp>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <lie.hpp>

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "imu_filter");
    ros::NodeHandle nh;

    auto useOdomFrame = nh.param<bool>("use_odom", false);
    auto roverFrame = nh.param<std::string>("rover_frame", "base_link");
    auto mapFrame = nh.param<std::string>("map_frame", "map");

    if (useOdomFrame) throw std::runtime_error{"Not supported"};

    tf2_ros::TransformBroadcaster tfBroadcaster;

    ros::Publisher odometryPub = nh.advertise<nav_msgs::Odometry>("/odometry", 1);

    std::optional<SE3d> lastPose;
    std::optional<ros::Time> lastPoseTime;

    ros::Subscriber poseSubscriber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/linearized_pose", 1, [&](geometry_msgs::PoseWithCovarianceStampedConstPtr const& poseMsg) {
        SE3d pose = SE3Conversions::fromPose(poseMsg->pose.pose);

        SE3Conversions::pushToTfTree(tfBroadcaster, roverFrame, mapFrame, pose);

        SE3d::Tangent twist;
        if (lastPose && lastPoseTime) {
            twist = (pose - lastPose.value()) / (poseMsg->header.stamp - lastPoseTime.value()).toSec();
        }

        nav_msgs::Odometry odometry;
        odometry.header = poseMsg->header;
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
        lastPoseTime = poseMsg->header.stamp;
    });

    ros::spin();

    return EXIT_SUCCESS;
}