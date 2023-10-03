#pragma once

namespace mrover {

    class LongRangeCamNodelet : public nodelet::Nodelet {
    
    private:
        cv::VideoCapture mCapture;

        struct Measures {
            ros::Time time;
            cv::OutputArray mImage;

            Measures() = default;

            Measures(Measures&) = delete;
            Measures& operator=(Measures&) = delete;

            Measures(Measures&&) noexcept;
            Measures& operator=(Measures&&) noexcept;
        };

        ros::NodeHandle mNh, mPnh;

        ros::Publisher mCamInfoPub;
        image_transport::Publisher mImgPub;

        Measures mGrabMeasures, mPubMeasures;

        std::thread mReadThread;
        std::mutex mSwapMutex;
        std::condition_variable mSwapCv;
        std::atomic_bool mIsSwapReady = false;

        LoopProfiler mGrabThreadProfiler{"Long Range Cam Grab"};

        size_t mGrabUpdateTick = 0;

        void onInit() override;

    public:
        LongRangeCamNodelet() = default;

        ~LongRangeCamNodelet() override;

        void readUpdate();
        void imagePubUpdate();
    };

    void fillCameraInfoMessages(sl::CalibrationParameters& calibration, sl::Resolution const& resolution,
                                sensor_msgs::CameraInfoPtr const& leftInfoMsg, sensor_msgs::CameraInfoPtr const& rightInfoMsg);

    void fillImageMessage(sl::Mat& bgra, sensor_msgs::ImagePtr const& msg);

} // namespace mrover
