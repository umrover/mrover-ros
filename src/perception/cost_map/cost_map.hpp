#pragma once

#include "pch.hpp"

namespace mrover {

    class CostMapNodelet final : public nodelet::Nodelet {

        constexpr static std::int8_t UNKNOWN_COST = -1, FREE_COST = 0, OCCUPIED_COST = 100;

        constexpr static double TAU = 2 * std::numbers::pi;

        ros::NodeHandle mNh, mPnh;
        ros::Publisher mCostMapPub;
        ros::Subscriber mPcSub;

        // TODO(quintin): This is a hack
        ros::Subscriber mImuSub;
        std::optional<ros::Time> mLastImuTime;

        float mZPercent{}, mZThreshold{};
        double mAlpha{};
        float mNearClip{}, mFarClip{};
        float mResolution{}; // Meters per cell
        float mSize{};       // Size of the square costmap in meters
        int mDownSamplingFactor = 4;
        std::string mMapFrame;

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
