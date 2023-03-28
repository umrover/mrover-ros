#include <optional>
#include <string>
#include <unordered_map>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

#include <mrover/DetectorParamsConfig.h>

#include "filter.hpp"
#include "se3.hpp"

using PointCloud = pcl::PointCloud<pcl::PointXYZRGBNormal>;
using PointCloudPtr = std::shared_ptr<PointCloud>;

struct Tag {
    int id = -1;
    int hitCount = 0;
    cv::Point2f imageCenter{};
    std::optional<SE3> tagInCam;
};

class FiducialsNode {
private:
    ros::NodeHandle mNh;
    ros::NodeHandle mPnh;

    image_transport::Publisher mImgPub;
    ros::ServiceServer mServiceEnableDetections;

    ros::Subscriber mIgnoreSub;
    ros::Subscriber mPcSub;
    image_transport::Subscriber mImgSub;
    image_transport::ImageTransport mIt;
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;
    tf2_ros::TransformBroadcaster mTfBroadcaster;

    bool mUseOdom = false;
    std::string mOdomFrameId, mMapFrameId, mBaseLinkFrameId;
    bool mPublishImages = false; // If set, we publish the images with the fiducials drawn on top
    bool mEnableDetections = true;
    bool mIsVerbose = false;
    bool mPublishFiducialTf = false;
    int mMinHitCountBeforePublish = 5;
    int mMaxHitCount = 10;

    std::vector<int> mIgnoreIds;
    cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
    cv::Ptr<cv::aruco::Dictionary> mDictionary;

    PointCloudPtr mCloudPtr = std::make_shared<PointCloud>();
    uint32_t mSeqNum{};
    std::optional<size_t> mPrevDetectedCount; // Log spam prevention
    std::vector<std::vector<cv::Point2f>> mCorners;
    std::vector<int> mIds;
    std::unordered_map<int, Tag> mTags;
    dynamic_reconfigure::Server<mrover::DetectorParamsConfig> mConfigServer;
    dynamic_reconfigure::Server<mrover::DetectorParamsConfig>::CallbackType mCallbackType;

    void handleIgnoreString(std::string const& str);

    void ignoreCallback(std_msgs::String const& msg);

    void imageCallback(sensor_msgs::ImageConstPtr const& msg);

    void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg);

    void configCallback(mrover::DetectorParamsConfig& config, uint32_t level);

    bool enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

public:
    FiducialsNode();
};
