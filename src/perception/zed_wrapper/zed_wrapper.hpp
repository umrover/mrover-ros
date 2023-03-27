#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include <sl/Camera.hpp>

#include <image_transport/publisher.h>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tag_detector.hpp>

#include "time_profiler.hpp"

namespace mrover {

    class ZedNodelet : public nodelet::Nodelet {
    private:
        ros::NodeHandle mNh, mPnh;

        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        ros::Publisher mPcPub, mImuPub, mLeftCamInfoPub, mRightCamInfoPub;
        image_transport::Publisher mLeftImgPub, mRightImgPub;

        sensor_msgs::ImagePtr mLeftImgMsg = boost::make_shared<sensor_msgs::Image>();
        sensor_msgs::ImagePtr mRightImgMsg = boost::make_shared<sensor_msgs::Image>();
        sensor_msgs::PointCloud2Ptr mPointCloud = boost::make_shared<sensor_msgs::PointCloud2>();
        sensor_msgs::CameraInfoPtr mLeftCamInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();
        sensor_msgs::CameraInfoPtr mRightCamInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();

        sl::Resolution mImageResolution;
        int mGrabTargetFps{};
        int mDepthConfidence{};
        int mTextureConfidence{};
        bool mDirectTagDetection{};
        bool mUseBuiltinPosTracking{};

        sl::Camera mZed;
        sl::Mat mLeftImageMat;
        sl::Mat mRightImageMat;
        sl::Mat mPointCloudXYZMat;
        sl::Mat mPointCloudNormalMat;

        std::thread mPcThread;
        std::thread mGrabThread;
        std::mutex mGrabMutex;
        std::condition_variable mGrabDone;

        boost::shared_ptr<TagDetectorNodelet> mTagDetectorNode;

        TimeProfiler mPcThreadProfiler;
        TimeProfiler mGrabThreadProfiler;

        size_t mGrabUpdateTick = 0;
        size_t mPointCloudUpdateTick = 0;

        void onInit() override;

    public:
        ZedNodelet() = default;

        ~ZedNodelet() override;

        void grabUpdate();

        void pointCloudUpdate();
    };

    ros::Time slTime2Ros(sl::Timestamp t);

    void fillPointCloudMessage(sl::Mat& xyz, sl::Mat& bgr, sensor_msgs::PointCloud2Ptr const& msg);

    void fillCameraInfoMessages(sl::CalibrationParameters& calibration, sl::Resolution const& resolution,
                                sensor_msgs::CameraInfoPtr const& leftInfoMsg, sensor_msgs::CameraInfoPtr const& rightInfoMsg);

    void fillImageMessage(sl::Mat& bgr, sensor_msgs::ImagePtr const& msg);

    void fillImuMessage(sl::SensorsData::IMUData& imuData, sensor_msgs::Imu& msg);

} // namespace mrover
