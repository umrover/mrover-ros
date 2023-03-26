#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include <sl/Camera.hpp>

#include <image_transport/publisher.h>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
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
        ros::Publisher mPcPub, mImuPub;
        image_transport::Publisher mLeftImgPub, mRightImgPub;

        sensor_msgs::Image mLeftImgMsg, mRightImgMsg;
        sensor_msgs::PointCloud2Ptr mPointCloud = boost::make_shared<sensor_msgs::PointCloud2>();

        sl::Resolution mImageResolution;
        int mGrabTargetFps{};
        int mDepthConfidence{};
        int mTextureConfidence{};
        bool mDirectTagDetection{};

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

        size_t mUpdateTick = 0;

        void onInit() override;

    public:
        ZedNodelet() = default;

        ~ZedNodelet() override;

        void grabUpdate();

        void pointCloudUpdate();
    };

    void fillPointCloudMessage(sl::Mat& xyz, sl::Mat& bgr, sensor_msgs::PointCloud2Ptr const& msg, size_t tick);

    void fillImageMessage(sl::Mat& bgr, sensor_msgs::Image& msg, size_t tick);

    void fillImuMessage(sl::SensorsData::IMUData& imuData, sensor_msgs::Imu& msg, size_t tick);

} // namespace mrover
