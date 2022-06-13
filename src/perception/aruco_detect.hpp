#include <cmath>
#include <numeric>
#include <optional>
#include <string>
#include <unordered_map>

#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>
#include <opencv2/aruco.hpp>

#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <mrover/DetectorParamsConfig.h>

#include "filter.hpp"
#include "se3.h"

constexpr char const* ODOM_FRAME = "odom";
constexpr char const* ROVER_FRAME = "base_link";

struct ImmediateFiducial {
    int id = -1;
    cv::Point2f imageCenter{};
    std::optional<SE3> fidInCam;
};

struct PersistentFiducial {
    int id = -1;
    Filter<double> fidInOdomX;
    Filter<double> fidInOdomY;
    Filter<double> fidInOdomZ;

    void addReading(SE3 const& fidInCam);

    void setFilterParams(size_t count, double proportion);
};

class FiducialsNode {
private:
    ros::NodeHandle mNh;
    ros::NodeHandle mPnh;

    ros::Publisher mFidPub;
    image_transport::Publisher mImgPub;
    ros::ServiceServer mServiceEnableDetections;

    ros::Subscriber mCamInfoSub;
    ros::Subscriber mIgnoreSub;
    ros::Subscriber mPcSub;
    image_transport::Subscriber mImgSub;
    image_transport::ImageTransport mIt;
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTfListener;
    tf2_ros::TransformBroadcaster mTfBroadcaster;

    bool mPublishImages = false; // If set, we publish the images with the fiducials drawn on top
    bool mEnableDetections = true;
    bool mIsVerbose = false;
    bool mHasCamInfo = false;
    bool mPublishFiducialTf = false;
    double mFiducialLen{};
    std::vector<int> mIgnoreIds;
    int mFilterCount{};
    double mFilterProportion{};
    cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
    cv::Ptr<cv::aruco::Dictionary> mDictionary;

    int mSeqNum{};
    cv_bridge::CvImagePtr mCvPtr;
    cv::Mat mCamMat;
    cv::Mat mDistCoeffs;
    std::string mFrameId;
    std::optional<size_t> mPrevDetectedCount; // Log spam prevention
    std::vector<std::vector<cv::Point2f>> mCornersCache;
    std::vector<int> mIdsCache;
    std::unordered_map<int, ImmediateFiducial> mImmediateFiducials;
    std::unordered_map<int, PersistentFiducial> mPersistentFiducials;
    dynamic_reconfigure::Server<mrover::DetectorParamsConfig> mConfigServer;
    dynamic_reconfigure::Server<mrover::DetectorParamsConfig>::CallbackType mCallbackType;

    void handleIgnoreString(std::string const& str);

    void ignoreCallback(std_msgs::String const& msg);

    void imageCallback(sensor_msgs::ImageConstPtr const& msg);

    void pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg);

    void camInfoCallback(sensor_msgs::CameraInfo::ConstPtr const& msg);

    void configCallback(mrover::DetectorParamsConfig& config, uint32_t level);

    bool enableDetectionsCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

public:
    FiducialsNode();
};
