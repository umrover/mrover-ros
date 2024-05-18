#pragma once

#include "pch.hpp"

namespace mrover {
    //Data type for detection
    struct Detection {
        int classId{};
        std::string className;
        float confidence{};
        cv::Rect box;
    };

    class ObjectDetectorNodelet : public nodelet::Nodelet {

        std::string mModelName;

        LoopProfiler mLoopProfiler{"Object Detector", 1};
        static constexpr bool mEnableLoopProfiler = false;

        cv::Mat mImg;

        std::vector<std::string> classes{"bottle", "hammer"};

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
        std::string mMapFrame;

        std::vector<int> mObjectHitCounts{0, 0};

        int mObjIncrementWeight{};
        int mObjDecrementWeight{};
        int mObjHitThreshold{};
        int mObjMaxHitcount{};

        auto onInit() -> void override;

        auto getObjectInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr,
                                     size_t u, 
                                     size_t v, 
                                     size_t width, 
                                     size_t height) -> std::optional<SE3d>;

        auto spiralSearchInImg(sensor_msgs::PointCloud2ConstPtr const& cloudPtr,
                               size_t xCenter, 
                               size_t yCenter, 
                               size_t width, 
                               size_t height) -> std::optional<SE3d>;

        static auto convertPointCloudToRGBA(sensor_msgs::PointCloud2ConstPtr const& msg, cv::Mat& img) -> void;

        auto updateHitsObject(sensor_msgs::PointCloud2ConstPtr const& msg,
                              const std::vector<Detection>& detections, 
                              cv::Size const& imgSize = {640, 640}) -> void;

        auto publishImg(cv::Mat const& img) -> void;

        auto parseModelOutput(cv::Mat& output, 
                              std::vector<Detection>& detections, 
                              float modelScoreThreshold = 0.75, 
                              float modelNMSThreshold = 0.5) -> void;

        static auto drawOnImage(cv::Mat& image, const std::vector<Detection>& detections) -> void;

        auto modelForwardPass(cv::Mat& image, 
                              std::vector<Detection>& detections, 
                              float modelScoreThreshold = 0.75, 
                              float modelNMSThreshold = 0.5) -> void;

    public:
        ObjectDetectorNodelet() = default;

        ~ObjectDetectorNodelet() override = default;

        auto pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;

        auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void;
    };

} // namespace mrover
