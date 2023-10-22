#include "can_node.hpp"

#include <mutex>
#include <stdexcept>

#include <netlink/route/link.h>
#include <netlink/route/link/can.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>

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
            if ((mSocketFd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
                throw std::runtime_error("Failed to open socket");
            }

            ifreq ifr{};
            const char* interfaceName = "can0";
            strcpy(ifr.ifr_name, interfaceName);
            ioctl(mSocketFd, SIOCGIFINDEX, &ifr);

            sockaddr_can addr{
                    .can_family = AF_CAN,
                    .can_ifindex = ifr.ifr_ifindex,
            };

            if (bind(mSocketFd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
                throw std::runtime_error("Failed to bind to socket");
            }

            int enable_canfd = 1;
            if (setsockopt(mSocketFd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd)) < 0) {
                throw std::runtime_error("Failed to enable CAN FD");
            }

            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();
            mCanSubscriber = mNh.subscribe<CAN>("can_requests", 5, &CanNodelet::handleWriteMessage, this);
            mCanPublisher = mNh.advertise<mrover::CAN>("{INSERT TOPIC NAME}", 1);

            mIsExtendedFrame = mNh.param<bool>("is_extended_frame", true);

            boost::asio::io_service ios;
            mStream.emplace(ios);
            mStream->assign(mSocketFd);

            mStream->async_read_some(
                    boost::asio::buffer(&mReadFrame, sizeof(mReadFrame)),
                    [this](auto const& ec, auto const& bytes) { readFrame(ec, bytes); });

            ios.run();

        } catch (std::exception const& exception) {
            ROS_ERROR_STREAM(exception.what());
            ros::requestShutdown();
        }
    }


    void CanNodelet::readFrame(boost::system::error_code const& ec, std::size_t bytes_transferred) {
        // TODO(quintin) check ec
        if (ec.value() != boost::system::errc::success) {
            throw std::runtime_error(std::format("Failed to read frame. Reason: {}", ec.value()));
        }

        CAN msg;
        msg.bus = 0;
        msg.message_id = mReadFrame.can_id;
        std::memcpy(msg.data.data(), mReadFrame.data, mReadFrame.len);

        mCanPublisher.publish(msg);

        boost::asio::async_read(
                mStream.value(),
                boost::asio::buffer(&mReadFrame, sizeof(mReadFrame)),
                [this](boost::system::error_code const& ec, std::size_t bytes) {
                    readFrame(ec, bytes);
                });
    }

    void CanNodelet::writeFrame() {
        // TODO(quintin) Convert to async_write_some
        boost::asio::async_write(
                mStream.value(),
                boost::asio::buffer(&mWriteFrame, sizeof(mWriteFrame)),
                [](boost::system::error_code const& ec, std::size_t bytes) {
                    if (ec.value() != boost::system::errc::success) {
                        throw std::runtime_error(std::format("Failed to write frame. Reason: {}", ec.value()));
                    }
                });
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
