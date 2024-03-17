#include "click_ik.hpp"

namespace mrover {

    void ClickIkNodelet::startClickIk() {
        ROS_INFO("Executing goal");

        const mrover::ClickIkGoalConstPtr& goal = server.acceptNewGoal();
        auto target_point = spiralSearchInImg(static_cast<size_t>(goal->pointInImageX), static_cast<size_t>(goal->pointInImageY));

        //Check if optional has value
        if (!target_point.has_value()) {
            //Handle gracefully
            ROS_WARN("Target point does not exist.");
            return;
        }

        geometry_msgs::Pose pose;
        
        double offset = 0.1; // make sure we don't collide by moving back a little from the target
        pose.position.x = target_point.value().x - offset;
        pose.position.y = target_point.value().y;
        pose.position.z = target_point.value().z;

        message.target.pose = pose;
        message.target.header.frame_id = "zed_left_camera_frame";
        timer = mNh.createTimer(ros::Duration(20 / 1000.0), [&](const ros::TimerEvent) {
            if (server.isPreemptRequested()) {
                timer.stop();
                return;
            }
            // Check if done
            const float tolerance = 0.02; // 2 cm
            SE3d arm_position = SE3Conversions::fromTfTree(mTfBuffer, "arm_e_link", "zed_left_camera_frame");
            double distance = pow(pow(arm_position.x() + ArmController::END_EFFECTOR_LENGTH - pose.position.x, 2) + pow(arm_position.y() - pose.position.y, 2) + pow(arm_position.z() - pose.position.z, 2), 0.5);
            ROS_INFO("Distance to target: %f", distance);
            if (distance < tolerance) {
                timer.stop();
                mrover::ClickIkResult result;
                result.success = true;
                server.setSucceeded(result);
                return;
            }
            // Otherwise publish message
            mIkPub.publish(message);
        });
    }

    void ClickIkNodelet::cancelClickIk() {
        timer.stop();
    }
    
    void ClickIkNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        mPcSub = mNh.subscribe("camera/left/points", 1, &ClickIkNodelet::pointCloudCallback, this);
        // IK Publisher
        mIkPub = mNh.advertise<IK>("/arm_ik", 1);

        ROS_INFO("Starting action server");
        server.registerGoalCallback([this]() { startClickIk(); });
        server.registerPreemptCallback([this] { cancelClickIk(); });
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
