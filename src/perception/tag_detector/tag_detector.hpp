#pragma once

#include "pch.hpp"

namespace mrover {

    struct Tag {
        int id = -1;
        int hitCount = 0;
        cv::Point2f imageCenter{};
        std::optional<SE3d> tagInCam;
    };

    class TagDetectorNodeletBase : public nodelet::Nodelet {

    protected:
        ros::NodeHandle mNh, mPnh;

        ros::Publisher mImgPub;
        std::unordered_map<int, ros::Publisher> mThreshPubs; // Map from threshold scale to publisher
        ros::ServiceServer mServiceEnableDetections;

        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;

        bool mEnableDetections = true;
        std::string mMapFrameId, mCameraFrameId;
        int mMinHitCountBeforePublish{};
        int mMaxHitCount{};
        int mTagIncrementWeight{};
        int mTagDecrementWeight{};

        cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mDictionary;

        cv::Mat mBgrImage, mGrayImg;
        sensor_msgs::Image mImgMsg;
        sensor_msgs::Image mThreshMsg;
        std::optional<size_t> mPrevDetectedCount; // Log spam prevention
        std::vector<std::vector<cv::Point2f>> mImmediateCorners;
        std::vector<int> mImmediateIds;
        std::unordered_map<int, Tag> mTags;
        dynamic_reconfigure::Server<DetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<DetectorParamsConfig>::CallbackType mCallbackType;

        LoopProfiler mProfiler{"Tag Detector"};

        auto onInit() -> void override;

        auto publishThresholdedImage() -> void;

        auto publishDetectedTags() -> void;

    public:
        TagDetectorNodeletBase() = default;

        ~TagDetectorNodeletBase() override = default;

        auto configCallback(DetectorParamsConfig const& config, uint32_t level) const -> void;

        auto enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) -> bool;
    };

    class StereoTagDetectorNodelet final : public TagDetectorNodeletBase {
        ros::Subscriber mPcSub;

        auto onInit() -> void override;

        auto pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;

        auto getTagInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, std::size_t u, std::size_t v) const -> std::optional<SE3d>;
    };

    class ImageTagDetectorNodelet final : public TagDetectorNodeletBase {
        ros::Subscriber mImgSub;

        ros::Publisher mTargetsPub;

        float mCameraHorizontalFov{};

        auto onInit() -> void override;

        auto getTagBearing(cv::InputArray image, std::span<cv::Point2f> const& tagCorners) const -> float;

        auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void;
    };

} // namespace mrover
