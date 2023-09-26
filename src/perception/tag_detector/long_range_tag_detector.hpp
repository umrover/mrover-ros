#include <opencv2/core/types.hpp>
namespace mrover {

    typedef struct LongRangeTagType {
        bool updated = false;
        int id = -1;
        int hitCount = 0;
        cv::Point2f imageCenter{};
    } LongRangeTag;

    class LongRangeTagDetectorNodelet : public nodelet::Nodelet {
        ros::NodeHandle mNh, mPnh;

        std::optional<image_transport::ImageTransport> mIt;
        image_transport::Publisher mImgPub;
        std::unordered_map<int, image_transport::Publisher> mThreshPubs; // Map from threshold scale to publisher
        ros::ServiceServer mServiceEnableDetections;

        image_transport::Subscriber mImgSub;

        bool mEnableDetections = true;
        bool mUseOdom{};
        std::string mOdomFrameId, mMapFrameId, mCameraFrameId;
        bool mPublishImages{}; // If set, we publish the images with the tags drawn on top
        int mMinHitCountBeforePublish{};
        int mMaxHitCount{};
        int mBaseHitCount{}; //Value the tag is initialized with after first detection. (Not incremented)
        int mTagIncrementWeight{};
        int mTagDecrementWeight{};
        int mTagRemoveWeight{}; //weight value before the tag is removed

        //ARUCO TAG DETECTION VARIABLES

        cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mDictionary;

        //IMAGE MESSAGE VARIABLES

        cv::Mat mImg;
        cv::Mat mGrayImg;
        sensor_msgs::Image mImgMsg;
        sensor_msgs::Image mThreshMsg;


        uint32_t mSeqNum{};
        std::optional<size_t> mPrevDetectedCount; // Log spam prevention

        //Raw Tag Data from CV::ARUCO
        std::vector<std::vector<cv::Point2f>> mImmediateCorners;
        std::vector<int> mImmediateIds;

        //Map from tagID to Tag
        std::unordered_map<int, LongRangeTag> mTags;
        dynamic_reconfigure::Server<mrover::DetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<mrover::DetectorParamsConfig>::CallbackType mCallbackType;

        LoopProfiler mProfiler{"Tag Detector"};

        void onInit() override;

        void publishThresholdedImage();

        std::optional<SE3> getTagInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t u, size_t v);


        /**
        * Detects tags in an image, draws the detected markers onto the image, and publishes them to /long_range_tag_detection
        * 1. Updates mImg to store the underlying image matrix
        * @param msg   image message
        */
        void imageCallback(sensor_msgs::Image const& msg);

        /**
        * Stores msg to mImgMsg
        * Creates image matrix from message, stores to mImg
        *
        * @param msg Current Image Message
        */
        void updateImageMatrices(sensor_msgs::Image const& msg);

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

        LongRangeTag createLrt(int tagId, std::vector<cv::Point2f>& tagCorners);

        /**
        *  Assumes that mImg exists
        *  @return a cv::Point2f that contains the centerx and centery
        * centerx and centery are -0.5, 0.5, and reflect a normalized offset from the image center
        * Average all x values
        */
        cv::Point2f getImageCenter(std::vector<cv::Point2f>& tagCorners);
    };
}; // namespace mrover