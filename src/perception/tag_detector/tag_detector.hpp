#include <optional>
#include <string>
#include <unordered_map>

#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <mrover/DetectorParamsConfig.h>

#include <se3.hpp>

#include "loop_profiler.hpp"

namespace mrover {

    struct Tag {
        int id = -1;
        int hitCount = 0;
        cv::Point2f imageCenter{};
        std::optional<SE3> tagInCam;
    };

    class TagDetectorNodelet : public nodelet::Nodelet {
    private:
        ros::NodeHandle mNh, mPnh;

        std::optional<image_transport::ImageTransport> mIt;
        image_transport::Publisher mImgPub;
        std::unordered_map<int, image_transport::Publisher> mThreshPubs; // Map from threshold scale to publisher
        ros::ServiceServer mServiceEnableDetections;

        ros::Subscriber mPcSub;
        image_transport::Subscriber mImgSub;
        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;

        bool mEnableDetections = true;
        bool mUseOdom{};
        std::string mOdomFrameId, mMapFrameId, mCameraFrameId;
        bool mPublishImages{}; // If set, we publish the images with the tags drawn on top
        int mMinHitCountBeforePublish{};
        int mMaxHitCount{};

        cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mDictionary;

        cv::Mat mImg;
        cv::Mat mGrayImg;
        sensor_msgs::Image mImgMsg;
        sensor_msgs::Image mThreshMsg;
        uint32_t mSeqNum{};
        std::optional<size_t> mPrevDetectedCount; // Log spam prevention
        std::vector<std::vector<cv::Point2f>> mCorners;
        std::vector<int> mIds;
        std::unordered_map<int, Tag> mTags;
        dynamic_reconfigure::Server<mrover::DetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<mrover::DetectorParamsConfig>::CallbackType mCallbackType;

        LoopProfiler mProfiler{"Tag Detector"};

        void onInit() override;

        void publishThresholdedImage();

        std::optional<SE3> getTagInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t u, size_t v);

    public:
        TagDetectorNodelet() = default;

        ~TagDetectorNodelet() override = default;

        void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg);

        void configCallback(mrover::DetectorParamsConfig& config, uint32_t level);

        bool enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    };

} // namespace mrover
