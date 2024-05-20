#pragma once

#include "pch.hpp"

namespace mrover {

    class UsbCameraNodelet final : public nodelet::Nodelet {

        ros::NodeHandle mNh, mPnh;

        ros::Publisher mCamInfoPub;
        ros::Publisher mImgPub;

        int mWidth{}, mHeight{};

        GstElement *mStreamSink{}, *mPipeline{};
        GMainLoop* mMainLoop{};

        std::thread mMainLoopThread, mStreamSinkThread;

        LoopProfiler mGrabThreadProfiler{"Long Range Cam Grab"};

        auto onInit() -> void override;

    public:
        UsbCameraNodelet() = default;

        ~UsbCameraNodelet() override;

        auto pullSampleLoop() const -> void;
    };

} // namespace mrover
