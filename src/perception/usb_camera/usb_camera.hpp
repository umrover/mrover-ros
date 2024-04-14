#pragma once

#include "pch.hpp"

namespace mrover {

    class UsbCameraNodelet : public nodelet::Nodelet {

        ros::NodeHandle mNh, mPnh;

        ros::Publisher mCamInfoPub;
        ros::Publisher mImgPub;

        std::jthread mGrabThread;

        LoopProfiler mGrabThreadProfiler{"Long Range Cam Grab"};

        auto onInit() -> void override;

    public:
        UsbCameraNodelet() = default;

        ~UsbCameraNodelet() override;

        auto grabUpdate() -> void;
    };

} // namespace mrover
