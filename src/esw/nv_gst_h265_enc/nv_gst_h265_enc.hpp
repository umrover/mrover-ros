#pragma once

#include "pch.hpp"

namespace mrover {

    class NvGstH265EncNodelet final : public nodelet::Nodelet {

        ros::NodeHandle mNh, mPnh;

        std::string mCaptureDevice;
        std::string mImageTopic;
        std::uint64_t mBitrate{};
        std::uint32_t mImageWidth{}, mImageHeight{}, mImageFramerate{};

        ros::Subscriber mImageSubscriber;

        std::optional<StreamServer> mStreamServer;

        GstElement *mImageSource{}, *mStreamSink{}, *mPipeline{};
        GMainLoop* mMainLoop{};
        std::thread mMainLoopThread;
        std::thread mStreamSinkThread;

        auto onInit() -> void override;

        auto pullStreamSamplesLoop() -> void;

        auto initPipeline() -> void;

        auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void;

        friend auto busMessageCallback(GstBus*, GstMessage* message, void* userData) -> gboolean;

    public:
        NvGstH265EncNodelet() = default;

        ~NvGstH265EncNodelet() override;
    };

} // namespace mrover
