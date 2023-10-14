#include "can_node.hpp"

#include <mutex>
#include <nodelet/loader.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/this_node.h>

namespace mrover {

    constexpr size_t CAN_ERROR_BIT_INDEX = 29;
    constexpr size_t CAN_RTR_BIT_INDEX = 30;
    constexpr size_t CAN_EXTENDED_BIT_INDEX = 31;

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

    void CanNodelet::setFrameData(std::span<const std::byte> data) {
        std::memcpy(mFrame.data, data.data(), data.size());
    }

    void CanNodelet::setFrameId(uint32_t identifier) {
        // Check that the identifier is not too large
        assert(std::bit_width(identifier) <= mIsExtendedFrame ? CAN_EFF_ID_BITS : CAN_SFF_ID_BITS);

        std::bitset<32> can_id_bits{identifier};
        if (mIsExtendedFrame) can_id_bits.set(CAN_EXTENDED_BIT_INDEX);
        can_id_bits.set(CAN_ERROR_BIT_INDEX);
        can_id_bits.set(CAN_RTR_BIT_INDEX);

        mFrame.can_id = can_id_bits.to_ullong();
    }

    void CanNodelet::handleMessage(CAN::ConstPtr const& msg) {
        std::scoped_lock lock(mMutex);

        setBus(msg->bus);
        setFrameId(msg->message_id);
        setFrameData({reinterpret_cast<std::byte const*>(msg->data.data()), msg->data.size()});

        sendFrame();
    }

    void CanNodelet::printFrame() {
        std::cout << std::format("BUS: {}", mBus) << std::endl;
        std::cout << std::format("CAN_ID: {}", mFrame.can_id) << std::endl;
        std::cout << std::format("LEN: {}", mFrame.len) << std::endl;
        std::cout << "DATA:" << std::endl;
        for (uint8_t i = 0; i < mFrame.len; ++i) {
            std::cout << std::format("Index = {}\tData = {:#X}", i, mFrame.data[i]) << std::endl;
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
