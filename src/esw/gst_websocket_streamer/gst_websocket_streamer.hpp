#pragma once

#include "pch.hpp"

// Uses gstreamer to encode and stream video over a websocket
// The input can be a ROS BGRA image topic or a USB device
// Hardware accelerated is used when possible (with the Jetson or NVIDIA GPUs)
// Run "export GST_DEBUG=2" to debug gstreamer issues

namespace mrover {

    struct ChunkHeader {
        enum struct Resolution : std::uint8_t {
            EGA, // 640x480
            HD,  // 1280x720
            FHD, // 1920x1080
        } resolution;
        enum struct Codec : std::uint8_t {
            H265,
            H264,
        } codec;
    };

    class GstWebsocketStreamerNodelet final : public nodelet::Nodelet {

        ros::NodeHandle mNh, mPnh;

        std::string mCaptureDevice;
        bool mDecodeJpegFromDevice{}; // Uses less USB hub bandwidth, which is limited since we are using 2.0
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
