#include "pch.hpp"

using manif::SE3d;

namespace mrover {

    struct Tag {
        int id = -1;
        int hitCount = 0;
        cv::Point2f imageCenter{};
        std::optional<SE3d> tagInCam;
    };

    class TagDetectorNodelet : public nodelet::Nodelet {
    private:
        ros::NodeHandle mNh, mPnh;

        ros::Publisher mImgPub;
        std::unordered_map<int, ros::Publisher> mThreshPubs; // Map from threshold scale to publisher
        ros::ServiceServer mServiceEnableDetections;

        ros::Subscriber mPcSub;
        ros::Subscriber mImgSub;
        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;

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

        std::optional<SE3d> getTagInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t u, size_t v);

    public:
        TagDetectorNodelet() = default;

        ~TagDetectorNodelet() override = default;

        void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg);

        void configCallback(mrover::DetectorParamsConfig& config, uint32_t level);

        bool enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

        // SE3d SE3fromTf(geometry_msgs::Transform const& transform);

        // SE3d SE3fromPose(geometry_msgs::Pose const& pose);

        // [[nodiscard]] geometry_msgs::Pose SE3toPose(SE3d const& tf);

        // [[nodiscard]] geometry_msgs::Transform SE3toTransform(SE3d const& tf);

        // [[nodiscard]] geometry_msgs::PoseStamped SE3toPoseStamped(SE3d const& tf, std::string const& frameId);

        // [[nodiscard]] geometry_msgs::TransformStamped SE3toTransformStamped(SE3d const& tf, std::string const& parentFrameId, std::string const& childFrameId);

        // [[nodiscard]] SE3d SE3fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId);

        // void pushSE3ToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3d const& tf);
    };

} // namespace mrover
