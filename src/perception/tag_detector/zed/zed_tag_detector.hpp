#pragma once

#include "../tag_detector.hpp"
#include "pch.hpp"

namespace mrover {

    struct Tag {
        int id = -1;
        int hitCount = 0;
        cv::Point2f imageCenter{};
        std::optional<SE3d> tagInCam;
    };

    class StereoTagDetectorNodelet : public TagDetector {
        std::unordered_map<int, ros::Publisher> mThreshPubs; // Map from threshold scale to publisher
        ros::ServiceServer mServiceEnableDetections;

        ros::Subscriber mPcSub;
        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;

        bool mUseOdom{};
        std::string mOdomFrameId, mMapFrameId, mCameraFrameId;
        int mMinHitCountBeforePublish{};
        int mMaxHitCount{};
        int mTagIncrementWeight{};
        int mTagDecrementWeight{};

        cv::Mat mImg;
        cv::Mat mGrayImg;cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mDictionary;
        sensor_msgs::Image mImgMsg;
        sensor_msgs::Image mThreshMsg;
        std::unordered_map<int, Tag> mTags;

        LoopProfiler mProfiler{"Tag Detector"};

        auto specificOnInit() -> void override;

        auto publishThresholdedImage() -> void;

        auto getTagInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t u, size_t v) const -> std::optional<SE3d>;

    public:
        StereoTagDetectorNodelet() = default;

        ~StereoTagDetectorNodelet() override = default;

        auto pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;
    };

} // namespace mrover
