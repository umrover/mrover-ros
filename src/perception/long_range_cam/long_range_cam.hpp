#pragma once

#include "pch.hpp"

namespace mrover {

    class LongRangeCamNodelet : public nodelet::Nodelet {

    private:
        cv::VideoCapture mCapture;

        ros::NodeHandle mNh, mPnh;

        ros::Publisher mCamInfoPub;
        ros::Publisher mImgPub;

        std::jthread mGrabThread;
        std::mutex mSwapMutex;
        boost::condition_variable mSwapCv;
        std::atomic_bool mIsSwapReady = false;

        LoopProfiler mGrabThreadProfiler{"Long Range Cam Grab"};

        size_t mGrabUpdateTick = 0;

        void onInit() override;

    public:
        LongRangeCamNodelet() = default;

        ~LongRangeCamNodelet() override;

        void grabUpdate();
    };

    void fillImageMessage(cv::Mat& grey, sensor_msgs::ImagePtr const& msg);

} // namespace mrover
