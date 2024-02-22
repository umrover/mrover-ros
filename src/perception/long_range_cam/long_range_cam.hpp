#pragma once

#include "pch.hpp"

namespace mrover {

    class LongRangeCamNodelet : public nodelet::Nodelet {

        ros::NodeHandle mNh, mPnh;

        ros::Publisher mCamInfoPub;
        ros::Publisher mImgPub;

        std::jthread mGrabThread;

        LoopProfiler mGrabThreadProfiler{"Long Range Cam Grab"};

        std::optional<StreamServer> streamServer;

        auto onInit() -> void override;

    public:
        LongRangeCamNodelet() = default;

        ~LongRangeCamNodelet() override;

        auto grabUpdate() -> void;
    };

} // namespace mrover
