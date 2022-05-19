/*
 * Copyright (c) 2017-20, Ubiquity Robotics Inc., Austin Hendrix
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are
 * those of the authors and should not be interpreted as representing official
 * policies, either expressed or implied, of the FreeBSD Project.
 *
 */


#include <cmath>
#include <unistd.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <dynamic_reconfigure/server.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/String.h>

#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialArray.h"
#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include "mrover/DetectorParamsConfig.h"

#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/ObjectHypothesisWithPose.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <list>
#include <string>
#include <numeric>
#include <boost/algorithm/string.hpp>
#include <boost/shared_ptr.hpp>

typedef boost::shared_ptr<fiducial_msgs::FiducialArray const> FiducialArrayConstPtr;

struct Fiducial {
    int id; // what id this ARUCO tag encodes
    cv::Point2f imageCenter; // camera pixel space
    cv::Point3f worldPosition; // 3d world position
};

class FiducialsNode {
private:
    ros::NodeHandle mNh;
    ros::NodeHandle mPnh;

    ros::Publisher mVerticesPub;
    ros::Publisher mPosePub;

    ros::Subscriber mCamInfoSub;
    ros::Subscriber mVerticesSub;
    ros::Subscriber mIgnoreSub;
    image_transport::ImageTransport mIt;
    image_transport::Subscriber mImgSub;
    ros::Subscriber mPcSub;
    tf2_ros::TransformBroadcaster mTfBroadcaster;

    ros::ServiceServer mServiceEnableDetections;

    bool mPublishImages{}; // if set, we publish the images that contain fiducials
    bool mEnableDetections;
    bool mIsVerbose{};

    std::vector<std::vector<cv::Point2f>> mCornersCache;
    std::vector<int> mIdsCache;
    double mFiducialLen{};

    bool mDoPoseEstimation{};
    bool mHasCamInfo;
    bool mPublishFiducialTf{};
    std::vector<Fiducial> mFiducials;
    cv_bridge::CvImagePtr mCvPtr;

    cv::Mat mCamMat;
    cv::Mat mDistCoeffs;
    int mFrameNum;
    std::string mFrameId;
    std::vector<int> mIgnoreIds;
    std::map<int, double> mFiducialLens;

    image_transport::Publisher mImgPub;

    size_t mPrevDetectedCount; // log spam prevention

    cv::Ptr<cv::aruco::DetectorParameters> mDetectorParams;
    cv::Ptr<cv::aruco::Dictionary> mDictionary;

    void handleIgnoreString(const std::string& str);

    void ignoreCallback(const std_msgs::String& msg);

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    void poseEstimateCallback(const FiducialArrayConstPtr& msg);

    void camInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);

    void configCallback(mrover::DetectorParamsConfig& config, uint32_t level);

    bool enableDetectionsCallback(std_srvs::SetBool::Request& req,
                                  std_srvs::SetBool::Response& res);

    dynamic_reconfigure::Server<mrover::DetectorParamsConfig> configServer;
    dynamic_reconfigure::Server<mrover::DetectorParamsConfig>::CallbackType callbackType;

public:
    FiducialsNode();
};
