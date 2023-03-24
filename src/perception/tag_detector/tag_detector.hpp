#include <optional>
#include <string>
#include <unordered_map>

#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <mrover/DetectorParamsConfig.h>

#include <se3.hpp>

namespace mrover {

    struct Tag {
        int id = -1;
        int hitCount = 0;
        cv::Point2f imageCenter{};
        std::optional<SE3> tagInCam;
    };

    class TagDetectorNode {
    private:
        ros::NodeHandle mNh, mPnh;

        image_transport::Publisher mImgPub;
        ros::ServiceServer mServiceEnableDetections;

        ros::Subscriber mPcSub;
        image_transport::Subscriber mImgSub;
        image_transport::ImageTransport mIt;
        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener;
        tf2_ros::TransformBroadcaster mTfBroadcaster;

        bool mUseOdom = false;
        std::string mOdomFrameId, mMapFrameId, mCameraFrameId;
        bool mPublishImages = false; // If set, we publish the images with the fiducials drawn on top
        bool mEnableDetections = true;
        int mMinHitCountBeforePublish = 5;
        int mMaxHitCount = 5;

        cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
        cv::Ptr<cv::aruco::Dictionary> mDictionary;

        cv::Mat mImg;
        sensor_msgs::Image mImgMsg;
        uint32_t mSeqNum{};
        std::optional<size_t> mPrevDetectedCount; // Log spam prevention
        std::vector<std::vector<cv::Point2f>> mCorners;
        std::vector<int> mIds;
        std::unordered_map<int, Tag> mTags;
        dynamic_reconfigure::Server<mrover::DetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<mrover::DetectorParamsConfig>::CallbackType mCallbackType;

    public:
        TagDetectorNode(ros::NodeHandle const& nh = {}, ros::NodeHandle const& pnh = {"~"});

        void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg);

        void configCallback(mrover::DetectorParamsConfig& config, uint32_t level);

        bool enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    };

    class TagDetectorNodelet : public nodelet::Nodelet {
    public:
        TagDetectorNodelet() = default;

        ~TagDetectorNodelet() override = default;

        TagDetectorNode* operator->();

    private:
        void onInit() override;

        boost::shared_ptr<TagDetectorNode> dtl;
    };

} // namespace mrover
