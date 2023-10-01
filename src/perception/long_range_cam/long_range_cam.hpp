#pragma once

namespace mrover {

    class LongRangeCamNodelet : public nodelet::Nodelet {
    
    private:
        explicit LongRangeCamNodelet;
        virtual ~LongRangeCamNodelet;
        LongRangeCamNodelet
        ros::NodeHandle mNh, mPnh;

        ros::Publisher mCamInfoPub;
        image_transport::Publisher mCamImgPub;

        PointCloudGpu mPointCloudGpu;

        sl::Resolution mImageResolution;
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

        Measures mGrabMeasures, mPcMeasures;

        std::thread mGrabThread;
        std::mutex mSwapMutex;
        std::condition_variable mSwapCv;
        std::atomic_bool mIsSwapReady = false;

        LoopProfiler mGrabThreadProfiler{"Long Range Cam Grab"};

        size_t mGrabUpdateTick = 0, mPointCloudUpdateTick = 0;

        void onInit() override;

    public:
        LongRangeCamNodelet() = default;

        ~LongRangeCamNodelet() override;

        void grabUpdate();
    };

    ros::Time slTime2Ros(sl::Timestamp t);

    void fillCameraInfoMessages(sl::CalibrationParameters& calibration, sl::Resolution const& resolution,
                                sensor_msgs::CameraInfoPtr const& leftInfoMsg, sensor_msgs::CameraInfoPtr const& rightInfoMsg);

    void fillImageMessage(sl::Mat& bgra, sensor_msgs::ImagePtr const& msg);

    void checkCudaError(cudaError_t error);

} // namespace mrover
