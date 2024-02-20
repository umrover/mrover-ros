#pragma once

#include "../tag_detector_class.hpp"
#include "pch.hpp"

namespace mrover {

    struct Tag {
        int id = -1;
        int hitCount = 0;
        cv::Point2f imageCenter{};
        std::optional<SE3> tagInCam;
    };

    class TagDetectorNodelet : public nodelet::Nodelet {
        // ros::NodeHandle mNh, mPnh;

        // ros::Publisher mImgPub;
        std::unordered_map<int, ros::Publisher> mThreshPubs; // Map from threshold scale to publisher
        ros::ServiceServer mServiceEnableDetections;

        ros::Subscriber mPcSub;
        // ros::Subscriber mImgSub;
        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;

        // bool mEnableDetections = true;
        bool mUseOdom{};
        std::string mOdomFrameId, mMapFrameId, mCameraFrameId;
        // bool mPublishImages{}; // If set, we publish the images with the tags drawn on top
        int mMinHitCountBeforePublish{};
        int mMaxHitCount{};
        int mTagIncrementWeight{};
        int mTagDecrementWeight{};

        cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mDictionary;

        cv::Mat mImg;
        cv::Mat mGrayImg;
        sensor_msgs::Image mImgMsg;
        sensor_msgs::Image mThreshMsg;
        // std::optional<size_t> mPrevDetectedCount; // Log spam prevention
        // std::vector<std::vector<cv::Point2f>> mImmediateCorners;
        // std::vector<int> mImmediateIds;
        std::unordered_map<int, Tag> mTags;
        // dynamic_reconfigure::Server<mrover::DetectorParamsConfig> mConfigServer;
        // dynamic_reconfigure::Server<mrover::DetectorParamsConfig>::CallbackType mCallbackType;

        LoopProfiler mProfiler{"Tag Detector"};

        auto onInit() override -> void override {
            TagDetector::onInit();
            mNh.param<bool>("use_odom_frame", mUseOdom, false);
            mNh.param<std::string>("odom_frame", mOdomFrameId, "odom");
            mNh.param<std::string>("world_frame", mMapFrameId, "map");
            mNh.param<std::string>("camera_frame", mCameraFrameId, "zed_left_camera_frame");

            mPnh.param<int>("dictionary", dictionaryNumber, static_cast<int>(cv::aruco::DICT_4X4_50));
            mPnh.param<int>("min_hit_count_before_publish", mMinHitCountBeforePublish, 5);
            mPnh.param<int>("max_hit_count", mMaxHitCount, 5);
            mPnh.param<int>("tag_increment_weight", mTagIncrementWeight, 2);
            mPnh.param<int>("tag_decrement_weight", mTagDecrementWeight, 1);

            mImgPub = mNh.advertise<sensor_msgs::Image>("tag_detection", 1);

            mPcSub = mNh.subscribe("camera/left/points", 1, &TagDetectorNodelet::pointCloudCallback, this);
        }

    void onInit() override {
        // Specific implementation for DerivedClass1
        BaseClass::onInit();
        std::cout << "Custom initialization for DerivedClass1 with " << additionalData << std::endl;
    }

        auto publishThresholdedImage() -> void;

        auto getTagInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t u, size_t v) const -> std::optional<SE3>;

    public:
        TagDetectorNodelet() = default;

        ~TagDetectorNodelet() override = default;

        auto pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void;

        // auto configCallback(DetectorParamsConfig& config, uint32_t level) -> void;

        // auto enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) -> bool;
    };

} // namespace mrover
