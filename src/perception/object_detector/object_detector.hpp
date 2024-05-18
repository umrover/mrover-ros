#pragma once

#include "pch.hpp"

namespace mrover {
    //Data type for detection
    class ObjectDetectorNodelet : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        std::string mCameraFrameId;
        std::string mMapFrame;

        dynamic_reconfigure::Server<ObjectDetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<ObjectDetectorParamsConfig>::CallbackType mCallbackType;

        // PointCloud Vars
        std::string mModelNamePC;

        LoopProfiler mLoopProfilerPC{"Object Detector", 1};
        static constexpr bool mEnableLoopProfilerPC = false;

        Learning mLearningPC;

        cv::Mat mImgPC;

        ros::Publisher mDebugImgPubPC;

        ros::Subscriber mImgSubPC;

        cv::Mat mImageBlobPC;

        // Image Vars
        std::string mModelNameIMG;

        LoopProfiler mLoopProfilerIMG{"Object Detector", 1};
        static constexpr bool mEnableLoopProfilerIMG = false;

        Learning mLearningIMG;

        cv::Mat mImgIMG;

        ros::Publisher mDebugImgPubIMG;

        ros::Subscriber mImgSubIMG;

        cv::Mat mImageBlobIMG;

        // Both PointCloud and Image

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

        static auto convertImageToRGBA(sensor_msgs::ImageConstPtr const& msg, cv::Mat& img) -> void;

        auto updateHitsObject(sensor_msgs::PointCloud2ConstPtr const& msg,
                              const std::vector<Detection>& detections, 
                              cv::Size const& imgSize = {640, 640}) -> void;

        auto publishImg(cv::Mat const& img) -> void;

        static auto drawOnImage(cv::Mat& image, const std::vector<Detection>& detections) -> void;

    public:
        ObjectDetectorNodelet() = default;

        ~ObjectDetectorNodelet() override = default;

        auto pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;

        auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void;
    };

} // namespace mrover
