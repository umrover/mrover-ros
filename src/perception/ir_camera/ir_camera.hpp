#pragma once
#include "pch.hpp"
#include <gst/gstelement.h>

namespace mrover {
    class IRCamera : public nodelet::Nodelet{
    private:
        ros::NodeHandle mNh, mPnh;

        int mWidth{};
        int mHeight{};
        int mFramerate{};

        double mRestartDelay{};

        GMainLoop* mMainLoop{};
        GstElement* mPipeline{};
        GstElement *mStreamSink{};

        std::thread mMainLoopThread, mStreamSinkThread; 

        ros::Timer mWatchdogTimer;

        ros::Publisher mImgPub;

        void pullSampleLoop();

        auto watchdogTriggered(ros::TimerEvent const&) -> void;

    public:
        auto onInit() -> void override;

        IRCamera() = default;

        ~IRCamera() override = default;
    };
}