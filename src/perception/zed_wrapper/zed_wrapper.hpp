#pragma once

#include "pch.hpp"

namespace mrover {

    using PointCloudGpu = thrust::device_vector<Point>;

    class ZedNodelet : public nodelet::Nodelet {
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
        ros::Publisher mPcPub, mImuPub, mMagPub, mLeftCamInfoPub, mRightCamInfoPub, mLeftImgPub, mRightImgPub;

        PointCloudGpu mPointCloudGpu;

        sl::Resolution mImageResolution, mPointResolution;
        sl::String mSvoPath;
        int mGrabTargetFps{};
        int mDepthConfidence{};
        int mTextureConfidence{};
        bool mUseBuiltinPosTracking{};
        bool mUseAreaMemory{};
        bool mUsePoseSmoothing{};
        bool mUseLoopProfiler{};
        bool mUseDepthStabilization{};
        float mDepthMaximumDistance{};

        sl::Camera mZed;
        sl::CameraInformation mZedInfo;
        Measures mGrabMeasures, mPcMeasures;

        std::thread mPointCloudThread, mGrabThread;
        std::mutex mSwapMutex;
        std::condition_variable mSwapCv;
        bool mIsSwapReady = false;

        LoopProfiler mPcThreadProfiler{"Zed Wrapper Point Cloud"}, mGrabThreadProfiler{"Zed Wrapper Grab"};

        size_t mGrabUpdateTick = 0, mPointCloudUpdateTick = 0;

        auto onInit() -> void override;

    public:
        ZedNodelet() = default;

        ~ZedNodelet() override;

        auto grabUpdate() -> void;

        auto pointCloudUpdate() -> void;
    };

    auto slTime2Ros(sl::Timestamp t) -> ros::Time;

    auto fillPointCloudMessageFromGpu(sl::Mat& xyzGpu, sl::Mat& bgraGpu, PointCloudGpu& pcGpu, sensor_msgs::PointCloud2Ptr const& msg) -> void;

    auto fillCameraInfoMessages(sl::CalibrationParameters& calibration, sl::Resolution const& resolution,
                                sensor_msgs::CameraInfoPtr const& leftInfoMsg, sensor_msgs::CameraInfoPtr const& rightInfoMsg) -> void;

    auto fillImageMessage(sl::Mat const& bgra, sensor_msgs::ImagePtr const& msg) -> void;

    auto fillImuMessage(sl::SensorsData::IMUData& imuData, sensor_msgs::Imu& msg) -> void;

    auto fillMagMessage(sl::SensorsData::MagnetometerData const& magData, sensor_msgs::MagneticField& msg) -> void;

    auto checkCudaError(cudaError_t error) -> void;

} // namespace mrover
