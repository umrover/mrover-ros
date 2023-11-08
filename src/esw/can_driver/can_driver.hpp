#pragma once

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <format>
#include <iostream>
#include <span>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/posix/basic_stream_descriptor.hpp>

#include "can_device.hpp"
#include <mrover/CAN.h>

#include "can_net_link.hpp"

// TODO(owen): support multiple buses

namespace mrover {

    // [0-28]: CAN identifier (11/29bit)
    // [29]: Error frame flag (0 = data frame, 1 = error frame)
    // [30]: Remote transmission request flag (1 = rtr frame)
    // [31]: Frame format flag (0 = standard 11bit, 1 = extended 29bit)
    // In the future, if we want to send different types of messages,
    // we should have logic for switching bits such as errorFrameFlag.
    struct RawCanFdId {
        std::uint32_t identifier : 29;
        bool isErrorFrame : 1;
        bool isRemoteTransmissionRequest : 1;
        bool isExtendedFrame : 1;
    };
    static_assert(sizeof(RawCanFdId) == sizeof(canid_t));

    class CanNodelet : public nodelet::Nodelet {
    public:
        CanNodelet() = default;

        ~CanNodelet() override;

    private:
        ros::NodeHandle mNh, mPnh;

        std::string mInterface;
        bool mIsExtendedFrame{};
        std::uint32_t mBitrate{};
        std::uint32_t mBitratePrescaler{};

        canfd_frame mReadFrame{};
        CanNetLink mCanNetLink;
        std::optional<boost::asio::posix::basic_stream_descriptor<>> mStream;
        std::jthread mIoThread;
        boost::asio::io_service mIoService;

        std::unordered_map<std::string, ros::Publisher> mCanPublisherByDeviceName;
        std::unordered_map<std::string, ros::Subscriber> mCanSubscriberByDeviceName;
        std::unordered_map<uint8_t, std::string> mDeviceNameByID; // TODO - use bimap
        std::unordered_map<std::string, uint8_t> mIDByDeviceName;

        // TODO - HERE IS AN EXAMPLE OF A BIMAP
        // bimap<std::string, CanFdDeviceAddress,
        //             std::hash<std::string>, decltype([](CanFdDeviceAddress const& location) {
        //                 return std::hash<std::uint8_t>{}(location.bus) ^ std::hash<std::uint8_t>{}(location.id);
        //             })>
        //                 m_devices;


        std::unordered_map<std::string, uint8_t> mBusByDeviceName;

        int setupSocket();

        void onInit() override;

        void readFrameAsync();

        void frameReadCallback();

        void frameSendRequestCallback(CAN::ConstPtr const& msg);
    };

} // namespace mrover
