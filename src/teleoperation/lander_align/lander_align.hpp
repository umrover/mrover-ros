#pragma once

#include "pch.hpp"

namespace mrover {

    class LanderAlignNodelet : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        ros::Publisher mDebugVectorPub;

        ros::Subscriber mVectorSub;

        //**
        float mBestOffset;

        Eigen::Vector3f mBestCenter;
        //**

        //TF Member Variables
        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        std::string mCameraFrameId;
        std::string mMapFrameId;

        float mThreshold;

        std::vector<Point const*> mFilteredPoints;

        auto onInit() -> void override;

        // deprecated/not needed anymore
        // auto downsample(sensor_msgs::PointCloud2Ptr const& cloud) -> sensor_msgs::PointCloud2Ptr;

        void LanderCallback(sensor_msgs::PointCloud2Ptr const& cloud);

        void filterNormals(sensor_msgs::PointCloud2Ptr const& cloud);

        auto ransac(std::vector<Point const*> const& points, float distanceThreshold, int minInliers, int epochs) -> std::optional<Eigen::Vector3f>;

        void sendTwist(Eigen::Vector3f const& mBestCenter);
    };

} // namespace mrover