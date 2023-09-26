namespace mrover {

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
        int mTagIncrementWeight{};
        int mTagDecrementWeight{};

        cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mDictionary;

        cv::Mat mImg;
        cv::Mat mGrayImg;
        sensor_msgs::Image mImgMsg;
        sensor_msgs::Image mThreshMsg;
        uint32_t mSeqNum{};
        std::optional<size_t> mPrevDetectedCount; // Log spam prevention
        std::vector<std::vector<cv::Point2f>> mImmediateCorners;
        std::vector<int> mImmediateIds;
        std::unordered_map<int, Tag> mTags;
        dynamic_reconfigure::Server<mrover::DetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<mrover::DetectorParamsConfig>::CallbackType mCallbackType;

        LoopProfiler mProfiler{"Tag Detector"};

        void onInit() override;

        void publishThresholdedImage();

        std::optional<SE3> getTagInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t u, size_t v);       
    }
}