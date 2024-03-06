#include <actionlib/server/action_server.h>
#include <actionlib/server/simple_action_server.h>
#include "click_ik.hpp"
#include "mrover/ClickIkAction.h"
#include "mrover/ClickIkGoal.h"
#include "mrover/IK.h"

#include <limits>

namespace mrover {

    void ClickIkNodelet::execute(const mrover::ClickIkGoalConstPtr& goal) {
        ROS_INFO("Executing goal");
    }
    
    void ClickIkNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mPcSub = mNh.subscribe("camera/left/points", 1, &ClickIkNodelet::pointCloudCallback, this);
        // IK Publisher
        mIkPub = mNh.advertise<IK>("/arm_ik", 1);

        server.start();

    }

    void ClickIkNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
        // Update current pointer to pointcloud data
        mPoints = reinterpret_cast<Point const*>(msg->data.data());
        mNumPoints = msg->width * msg->height;
    }

    auto ClickIkNodelet::spiralSearchInImg(size_t xCenter, size_t yCenter) -> std::optional<SE3d> {
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

        return std::make_optional<SE3d>(R3{point.x, point.y, point.z}, SO3d::Identity());
    }


    
} // namespace mrover
