#include "pch.hpp"

namespace mrover {

    class LongRangeTagDetectorNodelet : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        //Image Subscriber
        ros::Subscriber mImgSub;
        ros::Publisher mImgPub;

        //Publishes LongRangeTags messages
        ros::Publisher mLongRangeTagsPub;

        //Publishing Flags
        bool mEnableDetections = true;
        bool mPublishImages{}; // If set, we publish the images with the tags drawn on top

        // Camera lens intrinsic FOV
        float mLongRangeFov{};

        cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mDictionary;

        //IMAGE VARIABLES
        cv::Mat mImg;
        sensor_msgs::Image mImgMsg;

        //Raw Tag Data from CV::ARUCO
        std::vector<std::vector<cv::Point2f>> mImmediateCorners;
        std::vector<int> mImmediateIds;

        // Message header information
        std::optional<size_t> mPrevDetectedCount; // Log spam prevention
        dynamic_reconfigure::Server<DetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<DetectorParamsConfig>::CallbackType mCallbackType;
        LoopProfiler mProfiler{"Long Range Tag Detector"};
        ros::ServiceServer mServiceEnableDetections;

        std::string mMapFrameId, mCameraFrameId;

        auto onInit() -> void override;

        /**
        * Detects tags in an image, draws the detected markers onto the image, and publishes them to /long_range_tag_detection
        * 1. Updates mImg to store the underlying image matrix
        * @param msg   image message
        */
        auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void;

        /**
        * Given the known tag information and the Long range cam FOV, calculate relative bearing of a detected tag
        * @param tagCorners reference to tag corners
        * @return float of tag bearing
        */
        auto getTagBearing(std::vector<cv::Point2f>& tagCorners) const -> float;


        /**
        * publishes the thresholded tags onto an image using OpenCV
        * only if mPublishImages and the topic has a subscriber
        */
        auto publishTagsOnImage() -> void;

        auto configCallback(DetectorParamsConfig& config, uint32_t level) -> void;

        auto enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) -> bool;
    };

} // namespace mrover
