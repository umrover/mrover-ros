#pragma once

#include "pch.hpp"

namespace mrover {

    class ObjectDetectorNodeletBase : public nodelet::Nodelet {

    protected:
        ros::NodeHandle mNh, mPnh;

        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        std::string mCameraFrame;
        std::string mWorldFrame;

        dynamic_reconfigure::Server<ObjectDetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<ObjectDetectorParamsConfig>::CallbackType mCallbackType;

        std::string mModelName;

        LoopProfiler mLoopProfiler{"Object Detector", 1};

        Learning mLearning;

        cv::Mat mRgbImage, mImageBlob;
        sensor_msgs::Image mDetectionsImageMessage;

        ros::Publisher mDebugImagePub;

        ros::Subscriber mSensorSub;

        // TODO(quintin): Do not hard code exactly two classes
        std::vector<int> mObjectHitCounts{0, 0};

        int mObjIncrementWeight{};
        int mObjDecrementWeight{};
        int mObjHitThreshold{};
        int mObjMaxHitcount{};
        float mModelScoreThreshold{};
        float mModelNmsThreshold{};

        auto onInit() -> void override;

        auto spiralSearchForValidPoint(sensor_msgs::PointCloud2ConstPtr const& cloudPtr,
                                       std::size_t u, std::size_t v,
                                       std::size_t width, std::size_t height) const -> std::optional<SE3d>;

        auto updateHitsObject(sensor_msgs::PointCloud2ConstPtr const& msg,
                              std::span<Detection const> detections,
                              cv::Size const& imageSize = {640, 640}) -> void;

        auto publishDetectedObjects(cv::InputArray image) -> void;

        static auto drawDetectionBoxes(cv::InputOutputArray image, std::span<Detection const> detections) -> void;

    public:
        ObjectDetectorNodeletBase() = default;

        ~ObjectDetectorNodeletBase() override = default;
    };

    class StereoObjectDetectorNodelet final : public ObjectDetectorNodeletBase {
        auto onInit() -> void override;

        static auto convertPointCloudToRGB(sensor_msgs::PointCloud2ConstPtr const& msg, cv::Mat const& image) -> void;

        auto pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;
    };

    class ImageObjectDetectorNodelet final : public ObjectDetectorNodeletBase {
        ros::Publisher mTargetsPub;

        float mCameraHorizontalFov{};

        auto onInit() -> void override;

        auto getTagBearing(cv::InputArray image, cv::Rect const& box) const -> float;

        auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void;
    };

} // namespace mrover
