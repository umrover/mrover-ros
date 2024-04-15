#pragma once

#include "pch.hpp"

namespace mrover {

    class GstWebsocketStreamerNodelet final : public nodelet::Nodelet {

        ros::NodeHandle mNh, mPnh;

        // For example, /dev/video0
        // These device paths are not garunteed to stay the same between reboots
        // Prefer sys path for non-debugging purposes
        std::string mDeviceNode;
        std::string mImageTopic;
        // To find the sys path:
        // 1) Disconnect all cameras
        // 2) Confirm there are no /dev/video* devices
        // 2) Connect the camera you want to use
        // 3) Run "ls /dev/video*" to verify the device is connected
        // 4) Run "udevadm info -q path -n /dev/video0" to get the sys path
        std::string mDevicePath;
        std::uint64_t mBitrate{};
        std::uint32_t mImageWidth{}, mImageHeight{}, mImageFramerate{};

        ros::Subscriber mImageSubscriber;

        std::optional<WebsocketServer> mStreamServer;

        GstElement *mImageSource{}, *mStreamSink{}, *mPipeline{};
        GMainLoop* mMainLoop{};
        std::thread mMainLoopThread;
        std::thread mStreamSinkThread;

        auto onInit() -> void override;

        auto pullStreamSamplesLoop() -> void;

        auto initPipeline() -> void;

        auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void;

    public:
        GstWebsocketStreamerNodelet() = default;

        ~GstWebsocketStreamerNodelet() override;
    };

} // namespace mrover
