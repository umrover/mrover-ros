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

#include <mrover/CAN.h>

#include "can_net_link.hpp"

namespace mrover {

    struct FdcanId {
        std::uint32_t identifier : 29;
        bool isExtendedFrame : 1;
        bool isErrorFrame : 1;
        bool isRemoteTransmissionRequest : 1;
    };
    static_assert(sizeof(FdcanId) == sizeof(canid_t));

    class CanNodelet : public nodelet::Nodelet {
    public:
        CanNodelet() = default;

        ~CanNodelet() override;

        void readFrameAsync();

        void writeFrameAsync();

        void processReadFrame();

        void setBus(std::uint8_t bus);

        // canfd_frame.can_id is an uint32_t with format:
        // [0-28]: CAN identifier (11/29bit)
        // [29]: Error frame flag (0 = data frame, 1 = error frame)
        // [30]: Remote transmission request flag (1 = rtr frame)
        // [31]: Frame format flag (0 = standard 11bit, 1 = extended 29bit)
        // In the future, if we want to send different types of messages,
        // we should have logic for switching bits such as errorFrameFlag.
        void setFrameId(std::uint32_t identifier);

        void setFrameData(std::span<const std::byte> data);

    private:
        ros::NodeHandle mNh, mPnh;

        std::string mInterface;
        bool mIsExtendedFrame{};
        std::uint32_t mBitrate{};
        std::uint32_t mBitratePrescaler{};

        std::uint8_t mBus{};
        canfd_frame mReadFrame{};
        canfd_frame mWriteFrame{};
        CanNetLink mCanNetLink;
        std::optional<boost::asio::posix::basic_stream_descriptor<>> mStream;
        std::jthread mIoThread;
        boost::asio::io_service mIoService;

        ros::Publisher mCanPublisher;
        ros::Subscriber mCanSubscriber;

        int setupSocket();

        // Helper function for debug
        void printFrame();

        void onInit() override;

        void canSendRequestCallback(CAN::ConstPtr const& msg);
    };

} // namespace mrover

// 1 = high = recessive
// 0 = low = dominant

/*
struct canfd_frame {
    uint32_t can_id; // 32 bit CAN_ID + EFF/RTR/ERR flags
                     // [0-28]: CAN identifier (11/29bit)
                     // [29]: Error frame flag (0 = data frame, 1 = error frame)
                     // [30]: Remote transmission request flag (1 = rtr frame)
                     // [31]: Frame format flag (0 = standard 11bit, 1 = extended 29bit)

    uint8_t len;    // frame payload length in byte (0 .. 64)
    uint8_t flags;  // additional flags for CAN FD
    uint8_t __res0; // reserved / padding
    uint8_t __res1; // reserved / padding

    
    // REQUIREMENT: DATA MUST BE 0-8, 12, 16, 20, 24, 32, 36, 48, or 64 BYTES LONG
    uint8_t data[64];
};
*/