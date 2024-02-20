#pragma once

#include "pch.hpp"

namespace mrover {

    class CostMapNodelet final : public nodelet::Nodelet {

        constexpr static int UNKNOWN_COST = -1, FREE_COST = 0, OCCUPIED_COST = 1;

        ros::NodeHandle mNh, mPnh, mCmt;
        ros::Publisher mCostMapPub;
        ros::Subscriber mPcSub;

        bool mPublishCostMap{}; // If set, publish the global costmap
        float mResolution{};    // Meters per cell
        float mDimension{};     // Dimensions of the square costmap in meters
        float mNormalThreshold = 0.9;
        int mDownSamplingFactor = 4;
        uint32_t mNumPoints = 640 * 480 / mDownSamplingFactor;
        Eigen::MatrixXd mPointsInMap{mNumPoints, 4};
        Eigen::MatrixXd mNormalsInMap{mNumPoints, 4};

        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};

        std::optional<SE3d> mPreviousPose;
        nav_msgs::OccupancyGrid mGlobalGridMsg;

        void onInit() override;

    public:
        CostMapNodelet() = default;

        ~CostMapNodelet() override = default;

        auto pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;
    };

} // namespace mrover
