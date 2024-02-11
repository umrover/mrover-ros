#pragma once

#include "pch.hpp"

namespace mrover {
    //Data type for detection
    struct Detection {
        int classId{0};
        std::string className;
        float confidence{0.0};
        cv::Rect box;
    };

    class ObjectDetectorNodelet : public nodelet::Nodelet {

        std::string mModelName;

        LoopProfiler mLoopProfiler{"Object Detector", 1};
        bool mEnableLoopProfiler{};

        cv::Mat mImg;

        std::vector<std::string> classes{"Bottle", "Hammer"};

        ros::NodeHandle mNh, mPnh;

        InferenceWrapper mInferenceWrapper;

        ros::Publisher mDebugImgPub;

        ros::Subscriber mImgSub;

        cv::Mat mImageBlob;

        dynamic_reconfigure::Server<ObjectDetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<ObjectDetectorParamsConfig>::CallbackType mCallbackType;

        cv::dnn::Net mNet;

        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        std::string mCameraFrameId;
        std::string mMapFrameId;

        std::vector<int> mObjectHitCounts{0, 0};

        int mObjIncrementWeight{};
        int mObjDecrementWeight{};
        int mObjHitThreshold{};
        int mObjMaxHitcount{};

        auto onInit() -> void override;

        auto getObjectInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr,
                                     size_t u, size_t v, size_t width, size_t height) -> std::optional<SE3>;

        auto spiralSearchInImg(sensor_msgs::PointCloud2ConstPtr const& cloudPtr,
                               size_t xCenter, size_t yCenter, size_t width, size_t height) -> std::optional<SE3>;

        static auto convertPointCloudToRGBA(sensor_msgs::PointCloud2ConstPtr const& msg, cv::Mat& img) -> void;

        auto updateHitsObject(sensor_msgs::PointCloud2ConstPtr const& msg,
                              Detection const& detection, std::vector<bool>& seenObjects,
                              cv::Size const& imgSize = {640, 640}) -> void;

        auto publishImg(cv::Mat const& img) -> void;

    public:
        ObjectDetectorNodelet() = default;

        ~ObjectDetectorNodelet() override = default;

        auto imageCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;
    };

} // namespace mrover
