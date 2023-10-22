#pragma once

#include <bitset>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <format>
#include <iostream>
#include <mutex>
#include <ros/publisher.h>
#include <span>
#include <thread>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <mrover/CAN.h>

namespace mrover {

    class CanNodelet : public nodelet::Nodelet {
    public:
        void readFrame(boost::system::error_code const& ec, std::size_t bytes_transferred);

        void writeFrame() const;

        void setBus(uint8_t bus);

        // canfd_frame.can_id is a uint32_t with format:
        // [0-28]: CAN identifier (11/29bit)
        // [29]: Error frame flag (0 = data frame, 1 = error frame)
        // [30]: Remote transmission request flag (1 = rtr frame)
        // [31]: Frame format flag (0 = standard 11bit, 1 = extended 29bit)
        // In the future, if we want to send different types of messages,
        // we should have logic for switching bits such as errorFrameFlag.
        void setFrameId(uint32_t identifier);

        void setFrameData(std::span<const std::byte> data);

    private:
        ros::NodeHandle mNh, mPnh;

        uint8_t mBus{};
        canfd_frame mWriteFrame{};
        canfd_frame mReadFrame{};
        int mSocketFd{};
        bool mIsExtendedFrame{};

        ros::Publisher mCanPublisher;
        ros::Subscriber mCanSubscriber;

        std::mutex mMutex;

        std::optional<boost::asio::posix::basic_stream_descriptor<>> mStream;

        // Helper function for debug
        void printFrame();

        void onInit() override;

        void handleWriteMessage(CAN::ConstPtr const& msg);
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

CAN Frame Layout:

Start of Frame (SOF): 1 bit

Arbitration Field: 12 bits (for standard ID)

ID: 11 bits
RTR: 1 bit
Control Field: more bits compared to classic CAN due to extended DLC

IDE: 1 bit
r0: 1 bit
Extended Data Length (EDL): 1 bit
Bit Rate Switch (BRS): 1 bit
Error State Indicator (ESI): 1 bit
DLC: 4 bits
Data Field: up to 64 bytes in CAN FD (not limited to 8 bytes as in classic CAN)

CRC Field: variable size depending on the length of the Data Field

CRC Sequence
CRC Delimiter: 1 bit
Acknowledgment Field: 2 bits

ACK Slot: 1 bit
ACK Delimiter: 1 bit
End of Frame (EOF): 7 bits

Intermission: 3 bits

*/