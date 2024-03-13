#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>
#include "click_ik.hpp"
#include "lie.hpp"
#include "mrover/ClickIkAction.h"
#include "mrover/ClickIkGoal.h"
#include "mrover/IK.h"

#include <limits>
#include <optional>

namespace mrover {

    void ClickIkNodelet::execute(const mrover::ClickIkGoalConstPtr& goal) {
        ROS_INFO("Executing goal");

        auto target_point = spiralSearchInImg(static_cast<size_t>(goal->pointInImageX), static_cast<size_t>(goal->pointInImageY));

        //Check if optional has value
        if (!target_point.has_value()) {
            //Handle gracefully
            ROS_WARN("Target point does not exist.");
            return;
        }

        //Convert target_point (SE3) to correct frame
        // auto inverse_transform = SE3Conversions::fromTfTree(mTfBuffer, "chassis_link", "zed2i_left_camera_frame");
        // auto desired_transform = inverse_transform.inverse();
        // SE3d arm_target = desired_transform * target_point.value();
        geometry_msgs::Pose pose;
        
        double offset = 0.1; // make sure we don't collide by moving back a little from the target
        pose.position.x = target_point.value().x - offset;
        pose.position.y = target_point.value().y;// - normal_scale * target_point.value().normal_y;
        pose.position.z = target_point.value().z;// - normal_scale * target_point.value().normal_z;

        // pose.orientation.w = target_point.value().quat().w();
        // pose.orientation.x = target_point.value().quat().x();
        // pose.orientation.y = target_point.value().quat().y();
        // pose.orientation.z = target_point.value().quat().z();
        IK message;
        message.target.pose = pose;
        message.target.header.frame_id = "zed_left_camera_frame";
        mIkPub.publish(message);

        SE3d target_adjusted{{message.target.pose.position.x, message.target.pose.position.y, message.target.pose.position.z}, SO3d::Identity()};
        SE3d target_raw{{target_point.value().x, target_point.value().y, target_point.value().z}, SO3d::Identity()};
        SE3Conversions::pushToTfTree(mTfBroadcaster, "click_ik_target", "zed_left_camera_frame", target_adjusted);
        SE3Conversions::pushToTfTree(mTfBroadcaster, "click_ik_target_raw", "zed_left_camera_frame", target_raw);
        server.setSucceeded();
    }
    
    void ClickIkNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mPcSub = mNh.subscribe("camera/left/points", 1, &ClickIkNodelet::pointCloudCallback, this);
        // IK Publisher
        mIkPub = mNh.advertise<IK>("/arm_ik", 1);

        ROS_INFO("Starting action server");
        server.start();
        ROS_INFO("Action server started");

    }

    void ClickIkNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
        // Update current pointer to pointcloud data
        mPoints = reinterpret_cast<Point const*>(msg->data.data());
        mNumPoints = msg->width * msg->height;
        mPointCloudWidth = msg->width;
        mPointCloudHeight = msg->height;
    }

    auto ClickIkNodelet::spiralSearchInImg(size_t xCenter, size_t yCenter) -> std::optional<Point> {
        std::size_t currX = xCenter;
        std::size_t currY = yCenter;
        std::size_t radius = 0;
        int t = 0;
        constexpr int numPts = 16;
        bool isPointInvalid = true;
        Point point{};

        // Find the smaller of the two box dimensions so we know the max spiral radius
        std::size_t smallDim = std::min(mPointCloudWidth / 2, mPointCloudHeight / 2);

        while (isPointInvalid) {
            // This is the parametric equation to spiral around the center pnt
            currX = static_cast<size_t>(static_cast<double>(xCenter) + std::cos(t * 1.0 / numPts * 2 * M_PI) * static_cast<double>(radius));
            currY = static_cast<size_t>(static_cast<double>(yCenter) + std::sin(t * 1.0 / numPts * 2 * M_PI) * static_cast<double>(radius));

            // Grab the point from the pntCloud and determine if its a finite pnt
            point = mPoints[currX + currY * mPointCloudWidth];

            isPointInvalid = !std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z);
            if (isPointInvalid)
                NODELET_WARN("Tag center point not finite: [%f %f %f]", point.x, point.y, point.z);

            // After a full circle increase the radius
            if (t % numPts == 0) {
                radius++;
            }

            // Increase the parameter
            t++;

            // If we reach the edge of the box we stop spiraling
            if (radius >= smallDim) {
                return std::nullopt;
            }
        }

        return std::make_optional<Point>(point);
    }


    
} // namespace mrover
