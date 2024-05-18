#pragma once

#include "pch.hpp"

namespace mrover {

    class CostMapNodelet final : public nodelet::Nodelet {

        constexpr static std::int8_t UNKNOWN_COST = -1, FREE_COST = 0, OCCUPIED_COST = 1;

        ros::NodeHandle mNh, mPnh, mCmt;
        ros::Publisher mCostMapPub;
        ros::Subscriber mPcSub;

        bool mPublishCostMap{}; // If set, publish the global costmap
        float mResolution{};    // Meters per cell
        float mDimension{};     // Dimensions of the square costmap in meters
        double mNormalThreshold = 0.9;
        int mDownSamplingFactor = 4;
        std::uint32_t mNumPoints = 640 * 480 / mDownSamplingFactor / mDownSamplingFactor;
        Eigen::MatrixXf mPointsInMap{4, mNumPoints};
        Eigen::MatrixXf mNormalsInMap{4, mNumPoints};

        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};

        std::optional<SE3d> mPreviousPose;
        nav_msgs::OccupancyGrid mGlobalGridMsg;

        void onInit() override;

        ros::ServiceServer mServer;

    public:
        CostMapNodelet() = default;

        ~CostMapNodelet() override = default;

        auto pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;
        auto moveCostMapCallback(MoveCostMap::Request& req, MoveCostMap::Response& res) -> bool;
    };

} // namespace mrover
