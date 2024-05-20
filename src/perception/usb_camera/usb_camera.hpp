#pragma once

#include "pch.hpp"

namespace mrover {

    class UsbCameraNodelet final : public nodelet::Nodelet {

        ros::NodeHandle mNh, mPnh;

        ros::Publisher mCamInfoPub;
        ros::Publisher mImgPub;

        int mWidth{}, mHeight{};
        double mRestartDelay{};

        GstElement *mStreamSink{}, *mPipeline{};
        GMainLoop* mMainLoop{};

        std::thread mMainLoopThread, mStreamSinkThread;

        ros::Timer mWatchdogTimer;

        LoopProfiler mGrabThreadProfiler{"Long Range Cam Grab"};

        auto onInit() -> void override;

        auto watchdogTriggered(ros::TimerEvent const&) -> void;

    public:
        UsbCameraNodelet() = default;

        ~UsbCameraNodelet() override;

        auto pullSampleLoop() -> void;
    };

} // namespace mrover
