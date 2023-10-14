#include "can_node.hpp"

#include <nodelet/loader.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/this_node.h>

namespace mrover {

    void CanNodelet::onInit() {
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
        mCanSubscriber = mNh.subscribe<CAN>("can_requests", 1, &CanNodelet::handleMessage, this);

        mIsExtendedFrame = mNh.param<bool>("is_extended_frame", false);

        sockaddr_can addr{};
        ifreq ifr{};

        if ((mSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            perror("Error while opening socket");
        }

        const char* interfaceName = "can0";
        strcpy(ifr.ifr_name, interfaceName);
        ioctl(mSocket, SIOCGIFINDEX, &ifr);

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(mSocket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            perror("Error in socket bind");
        }
    }

    void CanNodelet::setBus(uint8_t bus) {
        this->mBus = bus;
    }

    void CanNodelet::sendFrame() const {
        // write(s, &frame, sizeof(struct can_frame));
        sendto(mSocket, &mFrame, sizeof(can_frame), 0, nullptr, 0);
    }

    void CanNodelet::setFrameData(std::span<std::byte> const& data) {
        // TODO(owen) this function needs to be thread safe. add a std::mutex and use std::scoped_lock
        // TODO(owen) change to std::memcpy
        for (size_t i = 0; i < data.size(); ++i) {
            mFrame.data[i] = std::to_integer<uint8_t>(data[i]);
        }
    }

    void CanNodelet::setFrameId(uint32_t id) {
        // TODO(owen) consider using std::bitset
        uint32_t frameFormatFlag;
        uint32_t identifier;
        if (mIsExtendedFrame) {
            frameFormatFlag = 0x80000000; // set can_id[31] high for extended frame
            identifier = id & 0x01FFFFFF; // lower 29 bit mask of id (extended)
        } else {
            frameFormatFlag = 0x00000000; // set can_id[31] low for standard frame
            identifier = id & 0x000007FF; // lower 11 bit mask of id (standard)
        }
        uint32_t errorFrameFlag = 0x20000000; // set can_id[29] high
        uint32_t rtrFlag = 0x40000000;        // set can_id[30] high

        mFrame.can_id = identifier | errorFrameFlag | rtrFlag | frameFormatFlag;
    }

    void CanNodelet::handleMessage(CAN::ConstPtr const& msg) {
        setBus(msg->bus);
        setFrameId(msg->message_id);
        // TODO(owen) currently not compiling
        //            setFrameData(std::span<std::byte>(reinterpret_cast<std::byte*>(msg->data.data()), msg->data.size()));

        sendFrame();
    }

    void CanNodelet::printFrame() {
        std::cout << std::format("BUS: {}", mBus) << std::endl;
        std::cout << std::format("CAN_ID: {}", mFrame.can_id) << std::endl;
        std::cout << std::format("LEN: {}", mFrame.len) << std::endl;
        std::cout << "DATA:" << std::endl;
        // TOOD(owen) change i to uint8_t and use hex formatter for std::format
        for (size_t i = 0; i < static_cast<size_t>(mFrame.len); ++i) {
            std::cout << std::format("Index = {}\tData = {}", i, unsigned(mFrame.data[i])) << std::endl;
        }
    }
} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_node");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/CanNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::CanNodelet, nodelet::Nodelet)
