#include "pch.hpp"
#include <sensor_msgs/Image.h>

namespace mrover {

    struct LongRangeTagStruct {
        bool updated = false;
        int id = -1;
        int hitCount = 0;
        cv::Point2f imageCenter{};
    };

    class LongRangeTagDetectorNodelet : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        //Image Subscriber
        ros::Subscriber mImgSub;
        ros::Publisher mImgPub;

        //Publishes LongRangeTags messages
        ros::Publisher mLongRangeTagsPub;

        //Publishing Flags
        bool mEnableDetections = true;
        // TODO: change this to param
        bool mPublishImages = true; // If set, we publish the images with the tags drawn on top

        //Constants set in ROS params for num hits needed to publish
        int mMinHitCountBeforePublish{};
        int mBaseHitCount{};
        int mMaxHitCount{};
        int mTagIncrementWeight{};
        int mTagDecrementWeight{};
        int mTagRemoveWeight{};
        float mLongRangeFov{};

        cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mDictionary;

        //IMAGE VARIABLES
        cv::Mat mImg;
        sensor_msgs::Image mImgMsg;

        //Raw Tag Data from CV::ARUCO
        std::vector<std::vector<cv::Point2f>> mImmediateCorners;
        std::vector<int> mImmediateIds;

        //Map from tagID to Tag
        std::unordered_map<int, LongRangeTagStruct> mTags;

        //TODO
        //All these variables are unused by long_range_tag_detector.processing.cpp
        //A bunch of things I think I should be able to delete
        //But I am not sure about
        uint32_t mSeqNum{};
        std::optional<size_t> mPrevDetectedCount; // Log spam prevention
        dynamic_reconfigure::Server<mrover::DetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<mrover::DetectorParamsConfig>::CallbackType mCallbackType;
        LoopProfiler mProfiler{"Long Range Tag Detector"};
        ros::ServiceServer mServiceEnableDetections;
        bool mUseOdom{};
        std::string mOdomFrameId, mMapFrameId, mCameraFrameId;

        void onInit() override;

        /**
        * Detects tags in an image, draws the detected markers onto the image, and publishes them to /long_range_tag_detection
        * 1. Updates mImg to store the underlying image matrix
        * @param msg   image message
        */
        void imageCallback(sensor_msgs::ImageConstPtr const& msg);

        /**
        * Stores msg to mImgMsg
        * Creates image matrix from message, stores to mImg
        *
        * @param msg Current Image Message
        */
        void updateImageMatrices(sensor_msgs::ImageConstPtr const& msg);

        /**
        * Runs the cv::aruco tag detection on the cv::mat 
        * Modifies mImmedateCorners and mImmediateIds
        * Uses the mDetectorParams and mDictionary member variables
        */
        void runTagDetection();

        /**
        * For all the tags stored in mImmediateIds and mImmediateCorners:
        * Check to see if they already exist in mTags
        * If they do, increment their 
        */
        void updateHitCounts();

        /**
        * @see updateNewlyIdentifiedTags()
        * @param tagId - the tagId of the current tag
        * @param tag_bearing - Bearing of the current tag
        * @return a new LongRangeTag
        */
        LongRangeTagStruct createLrt(int tagId, std::vector<cv::Point2f>& tagCorners);

        /**
        * @see getNormedTagCenter
        *  Assumes that mImg exists
        *  @return a cv::Point2f that contains the centerx and centery
        * centerx and centery are -0.5, 0.5, and reflect a normalized offset from the image center
        * Average all x values
        */
        cv::Point2f static getTagCenterPixels(std::vector<cv::Point2f>& tagCorners);

        /**
        * Assumes upper left corner is 0, 0
        * Want to return a negative pixel offset if width less than image_width / 2
        * similarily, negative if height > image_height / 2
        * @param tagCorners reference to tag corners, passed to @see getTagCenterPixesl
        * @return Point2f of pixel offset from center
        */
        cv::Point2f getTagCenterOffsetPixels(std::vector<cv::Point2f>& tagCorners) const;

        cv::Point2f getNormedTagCenterOffset(std::vector<cv::Point2f>& tagCorners) const;

        /**
        * Given the known tag information and the ZED FOV (hard code for now, we'll
        * assign params later), calculate relative bearing of a detected tag
        * @param tagCorners reference to tag corners, passed to @see getTagCenterPixels
        * @return float of tag bearing
        */
        float getTagBearing(cv::Point2f& tagCenter) const;

        /**
        * @see updateHitCounts()
        * Helper function used in updateHitCounts
        * Creates a new LongRangeTag for the identified tags and handles 
        * logic of adding it to the map
        * @param tagIndex the index i of the target tag in the mImmediate vectors
        */
        void updateNewlyIdentifiedTags(size_t tagIndex);


        /**
        * Publish the tags which have been detected for more than
        * mMinHitCountBeforePublish
        */
        void publishPermanentTags();

        /**
        * publishes the thresholded tags onto an image using OpenCV
        * only if mPublishImages and the topic has a subscriber
        */
        void publishTagsOnImage();

        void configCallback(mrover::DetectorParamsConfig& config, uint32_t level);

        bool enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    };
}; // namespace mrover
