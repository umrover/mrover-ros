#pragma once

#include "pch.hpp"

namespace mrover {

    class CostMapNodelet final : public nodelet::Nodelet {

        ros::NodeHandle mNh, mPnh, mCmt;
        ros::Publisher mCostMapPub;
        ros::Subscriber mPcSub;

        bool mPublishCostMap{}; // If set, publish the global costmap
        float mResolution{};    // Meters per cell
        float mDimension{};     // Dimensions of the square costmap in meters
        float mNormalThreshold = 0.9;
        int mDownSamplingFactor = 4;
        uint32_t mNumPoints = 640 * 480 / mDownSamplingFactor;
        Eigen::MatrixXd point_matrix{4, mNumPoints};
        Eigen::MatrixXd normal_matrix{4, mNumPoints};

        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};

        std::optional<SE3d> mPreviousPose;
        nav_msgs::OccupancyGrid mGlobalGridMsg;

        void onInit() override;

    public:
        CostMapNodelet() = default;

        ~CostMapNodelet() override = default;

        void configCallback();
        void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg);
    };

} // namespace mrover
