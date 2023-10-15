#include "can_node.hpp"

#include <mutex>
#include <nodelet/loader.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/this_node.h>
#include <stdexcept>

namespace mrover {

    constexpr size_t CAN_ERROR_BIT_INDEX = 29;
    constexpr size_t CAN_RTR_BIT_INDEX = 30;
    constexpr size_t CAN_EXTENDED_BIT_INDEX = 31;

    void CanNodelet::onInit() {
        try {
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();
            mCanSubscriber = mNh.subscribe<CAN>("can_requests", 1, &CanNodelet::handleMessage, this);

            mIsExtendedFrame = mNh.param<bool>("is_extended_frame", false);

            if ((mSocket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
                throw std::runtime_error("Failed to open socket");
            }

            ifreq ifr{};
            const char* interfaceName = "can0";
            strcpy(ifr.ifr_name, interfaceName);
            ioctl(mSocket, SIOCGIFINDEX, &ifr);

            sockaddr_can addr{
                    .can_family = AF_CAN,
                    .can_ifindex = ifr.ifr_ifindex,
            };


            if (bind(mSocket, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
                throw std::runtime_error("Failed to bind to socket");
            }
        } catch (std::exception const& exception) {
            ROS_ERROR_STREAM(exception.what());
            ros::requestShutdown();
        }
    }

    void CanNodelet::setBus(uint8_t bus) {
        this->mBus = bus;
    }

    // todo(owen): while loop until nbytes == size of can frame
    void CanNodelet::sendFrame() const {
        size_t nbytes = write(mSocket, &mFrame, sizeof(can_frame));
        if (nbytes < sizeof(struct can_frame)) {
            NODELET_INFO_STREAM("error: write incomplete CAN frame");
        }
        // sendto(mSocket, &mFrame, sizeof(can_frame), 0, nullptr, 0);
    }

    void CanNodelet::setFrameData(std::span<const std::byte> data) {
        std::memcpy(mFrame.data, data.data(), data.size());
        mFrame.len = data.size();
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

        // printFrame();

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

#if MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::CanNodelet, nodelet::Nodelet)

#else

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_node");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/CanNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#endif
