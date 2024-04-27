#pragma once

#include "pch.hpp"

namespace mrover {

    struct ChunkHeader {
        enum struct Resolution : std::uint8_t {
            EGA,
            HD,
            FHD,
        } resolution;
        enum struct Codec : std::uint8_t {
            H265,
            H264,
        } codec;
    };

    class GstWebsocketStreamerNodelet final : public nodelet::Nodelet {

        ros::NodeHandle mNh, mPnh;

        std::string mCaptureDevice;
        bool mDecodeJpegFromDevice{};
        std::string mImageTopic;
        std::uint64_t mBitrate{};
        std::uint32_t mImageWidth{}, mImageHeight{}, mImageFramerate{};

        ros::Subscriber mImageSubscriber;

        std::optional<WebsocketServer> mStreamServer;

        GstElement *mImageSource{}, *mStreamSink{}, *mPipeline{};
        GMainLoop* mMainLoop{};
        std::thread mMainLoopThread;
        std::thread mStreamSinkThread;

        ChunkHeader mChunkHeader{};

        auto onInit() -> void override;

        auto pullStreamSamplesLoop() -> void;

        auto initPipeline() -> void;

        auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void;

    public:
        GstWebsocketStreamerNodelet() = default;

        ~GstWebsocketStreamerNodelet() override;
    };

} // namespace mrover
