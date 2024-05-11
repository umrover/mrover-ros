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

static ros::Duration const STEP{0.5}, WINDOW{2.25};

constexpr static float MIN_LINEAR_SPEED = 0.2, MAX_ANGULAR_SPEED = 0.1, MIN_HEADING_STD_DEV = 0.2;

auto squared(auto const& x) { return x * x; }

auto wrapAngle(double angle) -> double {
    constexpr double pi = std::numbers::pi_v<double>;
    return std::fmod(angle + pi, 2 * pi) - pi;
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "heading_correcting_filter");
    ros::NodeHandle nh;

    auto use_odom = nh.param<bool>("use_odom", false);
    auto rover_frame = nh.param<std::string>("rover_frame", "base_link");
    auto map_frame = nh.param<std::string>("map_frame", "map");

    if (use_odom) throw std::runtime_error{"Not supported"};

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};

    tf2_ros::TransformBroadcaster tfBroadcaster;

    geometry_msgs::Twist currentTwist;

    S3 currentOrientation = S3::Identity(), correctionRotation = S3::Identity();

    ros::Timer correctTimer = nh.createTimer(WINDOW, [&](ros::TimerEvent const&) {
        // 1. Ensure the rover is being commanded to move relatively straight forward
        if (std::fabs(currentTwist.linear.x) < MIN_LINEAR_SPEED) return;
        if (std::fabs(currentTwist.angular.z) > MAX_ANGULAR_SPEED) return;

        ROS_INFO("Rover is being commanded forward");

        // Compute the past velocities and headings of the rover in the map frame over a window of time

        std::vector<R2> roverVelocitiesInMap;
        std::vector<double> roverHeadingsInMap;

        ros::Time end = ros::Time::now(), start = end - WINDOW;
        for (ros::Time t = start; t < end; t += STEP) {
            try {
                auto roverInMapOld = SE3Conversions::fromTfTree(tfBuffer, rover_frame, map_frame, t - STEP);
                auto roverInMapNew = SE3Conversions::fromTfTree(tfBuffer, rover_frame, map_frame, t);
                R2 roverVelocityInMap = (roverInMapNew.translation() - roverInMapOld.translation()).head<2>();
                roverVelocitiesInMap.push_back(roverVelocityInMap);
                roverHeadingsInMap.push_back(std::atan2(roverVelocityInMap.y(), roverVelocityInMap.x()));
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

        R2 meanVelocityInMap = std::accumulate(roverVelocitiesInMap.begin(), roverVelocitiesInMap.end(), R2{}) / roverVelocitiesInMap.size();
        if (meanVelocityInMap.norm() < MIN_LINEAR_SPEED) return;

        ROS_INFO("Rover is actually moving forward");

        // Angles wrap around so the mean + variance must be treated specially
        double meanHeadingInMap = std::atan2(meanVelocityInMap.y(), meanVelocityInMap.x());
        double stdDevHeadingInMap = std::sqrt(std::accumulate(roverHeadingsInMap.begin(), roverHeadingsInMap.end(), double{}, [&](double sum, double heading) {
                                                  return sum + squared(wrapAngle(heading - meanHeadingInMap));
                                              }) /
                                              static_cast<double>(roverHeadingsInMap.size()));

        // 3. Ensure the heading has not changed significantly (i.e. the rover is moving straight)

        if (stdDevHeadingInMap > MIN_HEADING_STD_DEV) {
            ROS_INFO_STREAM(std::format("Rover is not moving straight enough: std dev = {}", stdDevHeadingInMap));
            return;
        }

        ROS_INFO_STREAM("Heading corrected");
        correctionRotation = Eigen::AngleAxisd(meanHeadingInMap, R3::UnitZ()) * currentOrientation.inverse();
    });

    ros::Subscriber twistSubscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, [&](geometry_msgs::TwistConstPtr const& twist) {
        currentTwist = *twist;
    });

    ros::Subscriber poseSubscriber = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/linearized_pose", 1, [&](geometry_msgs::PoseWithCovarianceStampedConstPtr const& pose) {
        geometry_msgs::Quaternion const& o = pose->pose.pose.orientation;
        geometry_msgs::Point const& p = pose->pose.pose.position;
        currentOrientation = S3{o.w, o.x, o.y, o.z};
        S3 correctedOrientation = correctionRotation * currentOrientation;
        Eigen::Vector3d position{p.x, p.y, p.z};
        SE3Conversions::pushToTfTree(tfBroadcaster, rover_frame, map_frame, SE3d{position, correctedOrientation});
    });

    ros::spin();

    return EXIT_SUCCESS;
}