#pragma once

#include <condition_variable>
#include <mutex>
#include <thread>

#include <sl/Camera.hpp>
#include <thrust/device_vector.h>

#include <image_transport/publisher.h>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "../point_cloud.hpp"
#include "loop_profiler.hpp"

namespace mrover {

    using PointCloudGpu = thrust::device_vector<Point>;

    class ZedNodelet : public nodelet::Nodelet {
    private:
        struct Measures {
            ros::Time time;
            sl::Mat leftImage;
            sl::Mat rightImage;
            sl::Mat leftPoints;

            Measures() = default;

            Measures(Measures&) = delete;
            Measures& operator=(Measures&) = delete;

            Measures(Measures&&) noexcept;
            Measures& operator=(Measures&&) noexcept;
        };

        ros::NodeHandle mNh, mPnh;

        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        ros::Publisher mPcPub, mImuPub, mMagPub, mLeftCamInfoPub, mRightCamInfoPub;
        image_transport::Publisher mLeftImgPub, mRightImgPub;

        sensor_msgs::ImagePtr mLeftImgMsg = boost::make_shared<sensor_msgs::Image>();
        sensor_msgs::ImagePtr mRightImgMsg = boost::make_shared<sensor_msgs::Image>();
        PointCloudGpu mPointCloudGpu;
        sensor_msgs::PointCloud2Ptr mPointCloud = boost::make_shared<sensor_msgs::PointCloud2>();
        sensor_msgs::CameraInfoPtr mLeftCamInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();
        sensor_msgs::CameraInfoPtr mRightCamInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();

        sl::Resolution mImageResolution, mPointResolution;
        int mGrabTargetFps{};
        int mDepthConfidence{};
        int mTextureConfidence{};
        bool mUseOdom{};
        bool mUseBuiltinPosTracking{};

        sl::Camera mZed;
        sl::CameraInformation mZedInfo;
        Measures mGrabMeasures, mProcessMeasures;

        std::thread mProcessThread, mGrabThread;
        std::mutex mSwapMutex;
        std::condition_variable mSwapCv;
        std::atomic_bool mIsSwapReady = false;

        LoopProfiler mProcessThreadProfiler, mGrabThreadProfiler;

        size_t mGrabUpdateTick = 0, mPointCloudUpdateTick = 0;

        void onInit() override;

    public:
        ZedNodelet() = default;

        ~ZedNodelet() override;

        void grabUpdate();

        void pointCloudUpdate();
    };

    ros::Time slTime2Ros(sl::Timestamp t);

    void fillPointCloudMessage(sl::Mat& xyz, sl::Mat& bgra, PointCloudGpu& pcGpu, sensor_msgs::PointCloud2Ptr const& msg);

    void fillCameraInfoMessages(sl::CalibrationParameters& calibration, sl::Resolution const& resolution,
                                sensor_msgs::CameraInfoPtr const& leftInfoMsg, sensor_msgs::CameraInfoPtr const& rightInfoMsg);

    void fillImageMessage(sl::Mat& bgra, sensor_msgs::ImagePtr const& msg);

    void fillImuMessage(sl::SensorsData::IMUData& imuData, sensor_msgs::Imu& msg);

    void fillMagMessage(sl::SensorsData::MagnetometerData& magData, sensor_msgs::MagneticField& msg);

} // namespace mrover
