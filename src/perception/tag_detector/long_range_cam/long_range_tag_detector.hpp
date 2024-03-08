#include "../tag_detector.hpp"
#include "pch.hpp"

namespace mrover {

    class LongRangeTagDetectorNodelet : public TagDetector {
        //Publishes LongRangeTags messages
        ros::Publisher mLongRangeTagsPub;

        // Camera lens intrinsic FOV
        float mLongRangeFov{};

        cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mDictionary;

        //IMAGE VARIABLES
        cv::Mat mImg;
        sensor_msgs::Image mImgMsg;

        ros::ServiceServer mServiceEnableDetections;
                                                                                                
        std::string mMapFrameId, mCameraFrameId;

        auto specificOnInit() -> void override;
        // {
        //     mPnh.param<float>("long_range_fov", mLongRangeFov, 80.0);

        //     mImgPub = mNh.advertise<sensor_msgs::Image>("long_range_tag_detection", 1);
        //     mLongRangeTagsPub = mNh.advertise<LongRangeTags>("tags", 1);

        //     mImgSub = mNh.subscribe("long_range_image", 1, &LongRangeTagDetectorNodelet::imageCallback, this);
        // }

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
    };

} // namespace mrover

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::LongRangeTagDetectorNodelet, nodelet::Nodelet)
