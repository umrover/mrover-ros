#include "can_node.hpp"

#include <mutex>
#include <net/if.h>
#include <netlink/handlers.h>
#include <stdexcept>

#include <netlink/route/link.h>
#include <netlink/route/link/can.h>

#include <nodelet/loader.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/this_node.h>


struct Netlink {
    Netlink() {
        try {
            netLinkSocket = nl_socket_alloc();
            if (netLinkSocket == nullptr) {
                throw std::runtime_error("Failed to allocate netlink socket");
            }

            if (int status = nl_connect(netLinkSocket, NETLINK_ROUTE); status < 0) {
                throw std::runtime_error("Failed to connect to netlink socket");
            }

            nl_cache* cache;
            rtnl_link_alloc_cache(netLinkSocket, AF_UNSPEC, &cache);
            if (cache == nullptr) {
                throw std::runtime_error("Failed to allocate rtnl_link cache");
            }

            link = rtnl_link_get_by_name(cache, "can0");
            if (link == nullptr) {
                throw std::runtime_error("Failed to get rtnl_link");
            }

            can_bittiming bt{
                    .bitrate = 500000,
                    .brp = 2,
            };
            if (int result = rtnl_link_can_set_bittiming(link, &bt); result < 0) {
                throw std::runtime_error("Failed to set bittiming");
            }

            rtnl_link_set_flags(link, IFF_UP);
            if (int result = rtnl_link_change(netLinkSocket, link, link, 0); result < 0 && result != -NLE_SEQ_MISMATCH) {
                throw std::runtime_error(std::format("Failed to change rtnl_link: {}", result));
            }
        } catch (std::exception const& exception) {
            ROS_ERROR_STREAM(exception.what());
            ros::requestShutdown();
        }
    }
    ~Netlink() {
        try {
            rtnl_link_unset_flags(link, IFF_UP);
            if (int result = rtnl_link_change(netLinkSocket, link, link, 0); result < 0 && result != -NLE_SEQ_MISMATCH) {
                throw std::runtime_error(std::format("Failed to change rtnl_link: {}", result));
            }
        } catch (std::exception const& exception) {
            ROS_ERROR_STREAM(exception.what());
            ros::requestShutdown();
        }
    }

    nl_sock* netLinkSocket;
    rtnl_link* link;
};

namespace mrover {

    constexpr size_t CAN_ERROR_BIT_INDEX = 29;
    constexpr size_t CAN_RTR_BIT_INDEX = 30;
    constexpr size_t CAN_EXTENDED_BIT_INDEX = 31;

    Netlink nLink;

    void CanNodelet::onInit() {
        try {
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

            int enable_canfd = 1;
            if (setsockopt(mSocket, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
                throw std::runtime_error("Failed to enable CAN FD");
            }


            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();
            mCanSubscriber = mNh.subscribe<CAN>("can_requests", 5, &CanNodelet::handleWriteMessage, this);

            mIsExtendedFrame = mNh.param<bool>("is_extended_frame", true);

            boost::asio::io_service ios;
            boost::asio::posix::basic_stream_descriptor<> stream(ios);
            stream.assign(mSocket);


            stream.async_read_some(
                    boost::asio::buffer(&mReadFrame, sizeof(mReadFrame)),
                    boost::bind(&readFrame, this, boost::ref(mReadFrame), boost::ref(stream)));
            ios.run();


        } catch (std::exception const& exception) {
            ROS_ERROR_STREAM(exception.what());
            ros::requestShutdown();
        }
    }


    void CanNodelet::readFrame(struct can_frame& rec_frame,
                               boost::asio::posix::basic_stream_descriptor<>& stream) {

        std::cout << std::hex << rec_frame.can_id << "  ";
        for (int i = 0; i < rec_frame.can_dlc; i++) {
            std::cout << std::hex << int(rec_frame.data[i]) << " ";
        }
        std::cout << std::dec << std::endl;
        stream.async_read_some(
                boost::asio::buffer(&rec_frame, sizeof(rec_frame)),
                boost::bind(data_rec, boost::ref(rec_frame), boost::ref(stream)));
    }

    //todo(owen) Possible timeout mechanism? Maybe a limit of num writes before while look breaks and throws error
    void CanNodelet::writeFrame() const {
        size_t nbytes = write(mSocket, &mWriteFrame, sizeof(canfd_frame));
        while (nbytes != sizeof(struct canfd_frame)) {
            // nbytes = sendto(mSocket, &mFrame, sizeof(canfd_frame), 0, nullptr, 0);
            nbytes = write(mSocket, &mWriteFrame, sizeof(canfd_frame));
        }
        // NODELET_INFO_STREAM("error: write incomplete CAN frame");
    }

    void CanNodelet::setBus(uint8_t bus) {
        this->mBus = bus;
    }

    void CanNodelet::setFrameId(uint32_t identifier) {
        // Check that the identifier is not too large
        assert(std::bit_width(identifier) <= mIsExtendedFrame ? CAN_EFF_ID_BITS : CAN_SFF_ID_BITS);

        std::bitset<32> can_id_bits{identifier};
        if (mIsExtendedFrame) can_id_bits.set(CAN_EXTENDED_BIT_INDEX);
        can_id_bits.set(CAN_ERROR_BIT_INDEX);
        can_id_bits.set(CAN_RTR_BIT_INDEX);

        mWriteFrame.can_id = can_id_bits.to_ullong();
    }

    void CanNodelet::setFrameData(std::span<const std::byte> data) {
        std::memcpy(mWriteFrame.data, data.data(), data.size());
        mWriteFrame.len = data.size();
    }

    void CanNodelet::handleWriteMessage(CAN::ConstPtr const& msg) {
        std::scoped_lock lock(mMutex);

        setBus(msg->bus);
        setFrameId(msg->message_id);
        setFrameData({reinterpret_cast<std::byte const*>(msg->data.data()), msg->data.size()});

        writeFrame();
        // printFrame();
    }

    void CanNodelet::printFrame() {
        std::cout << std::format("BUS: {}", mBus) << std::endl;
        std::cout << std::format("CAN_ID: {}", mWriteFrame.can_id) << std::endl;
        std::cout << std::format("LEN: {}", mWriteFrame.len) << std::endl;
        std::cout << "DATA:" << std::endl;
        for (uint8_t i = 0; i < mWriteFrame.len; ++i) {
            std::cout << std::format("Index = {}\tData = {:#X}", i, mWriteFrame.data[i]) << std::endl;
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
