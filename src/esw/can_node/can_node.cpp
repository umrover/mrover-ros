#include "can_node.hpp"

#include <linux/can.h>
#include <stdexcept>

#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/system/error_code.hpp>

#include <nodelet/loader.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/this_node.h>

namespace mrover {

    constexpr static std::size_t CAN_ERROR_BIT_INDEX = 29;
    constexpr static std::size_t CAN_RTR_BIT_INDEX = 30;
    constexpr static std::size_t CAN_EXTENDED_BIT_INDEX = 31;

    void checkErrorCode(boost::system::error_code const& ec) {
        if (ec.value() == boost::system::errc::success) return;

        throw std::runtime_error(std::format("Boost failure: {}", ec.value()));
    }

    int checkSyscallResult(int result) {
        if (result < 0) throw std::system_error{errno, std::generic_category()};

        return result;
    }

    void CanNodelet::onInit() {
        try {
            NODELET_INFO("CAN Node starting...");

            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();

            mIsExtendedFrame = mNh.param<bool>("is_extended_frame", true);

            mCanSubscriber = mNh.subscribe<CAN>("can_requests", 16, &CanNodelet::canSendRequestCallback, this);
            mCanPublisher = mNh.advertise<mrover::CAN>("can_data", 16);

            mCanNetLink.emplace();

            int socketFd = setupSocket();
            mStream.emplace(mIoService);
            mStream->assign(socketFd);

            readFrameAsync();

            mIoThread = std::jthread{[this] {
                mIoService.run();
            }};

            NODELET_INFO("CAN Node started");

        } catch (std::exception const& exception) {
            NODELET_ERROR_STREAM("Exception in initialization: " << exception.what());
            ros::shutdown();
        }
    }

    int CanNodelet::setupSocket() {
        int socketFd = checkSyscallResult(socket(PF_CAN, SOCK_RAW, CAN_RAW));
        NODELET_INFO("Opened CAN socket");

        ifreq ifr{};
        const char* interfaceName = "can0";
        strcpy(ifr.ifr_name, interfaceName);
        ioctl(socketFd, SIOCGIFINDEX, &ifr);

        sockaddr_can addr{
                .can_family = AF_CAN,
                .can_ifindex = ifr.ifr_ifindex,
        };
        checkSyscallResult(bind(socketFd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)));
        NODELET_INFO("Bound CAN socket");

        int enableCanFd = 1;
        checkSyscallResult(setsockopt(socketFd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enableCanFd, sizeof(enableCanFd)));

        return socketFd;
    }

    void CanNodelet::readFrameAsync() { // NOLINT(*-no-recursion)

        // 1. Read the entire header to find the data size
        boost::asio::async_read(
                mStream.value(),
                boost::asio::buffer(&mReadHeader, sizeof(mReadHeader)),
                // This lambda ensures we read ALL the header in case it comes in chunks
                // It is a completion condition essentially
                [](boost::system::error_code const& ec, std::size_t bytes) {
                    checkErrorCode(ec);

                    return sizeof(mReadHeader);
                },
                // This lambda is called on completion
                [this](boost::system::error_code const& ec, std::size_t bytes) { // NOLINT(*-no-recursion)
                    checkErrorCode(ec);
                    assert(bytes == sizeof(mReadHeader));
                    mReadData.resize(mReadHeader.len);

                    // 2. Read the main data, we know how much to read from the header
                    boost::asio::async_read(
                            mStream.value(),
                            boost::asio::buffer(&mReadData, mReadData.size()),
                            [this](boost::system::error_code const& ec, std::size_t bytes) {
                                checkErrorCode(ec);

                                return mReadData.size();
                            },
                            [this](boost::system::error_code const& ec, std::size_t bytes) { // NOLINT(*-no-recursion)
                                checkErrorCode(ec);
                                assert(bytes == mReadData.size());

                                // 3. We have all the data associated with the frame, process it
                                processReadFrame();

                                // 4. Ready for the next frame, start listening
                                //    Note this is recursive, but it is safe because it is async
                                //    i.e. the stack is not growing
                                readFrameAsync();
                            });
                });
    }

    void CanNodelet::writeFrameAsync() {
        std::size_t toSend = mWriteFrame.len;
        boost::asio::async_write(
                mStream.value(),
                boost::asio::buffer(&mWriteFrame, toSend),
                [toSend](boost::system::error_code const& ec, std::size_t bytes) {
                    checkErrorCode(ec);
                    assert(bytes == toSend);
                });
    }

    void CanNodelet::processReadFrame() { // NOLINT(*-no-recursion)
        CAN msg;
        msg.bus = 0;
        msg.message_id = mReadHeader.can_id;
        msg.data = std::move(mReadData);

        mCanPublisher.publish(msg);
    }

    void CanNodelet::setBus(std::uint8_t bus) {
        this->mBus = bus;
    }

    void CanNodelet::setFrameId(std::uint32_t identifier) {
        // Check that the identifier is not too large
        assert(std::bit_width(identifier) <= mIsExtendedFrame ? CAN_EFF_ID_BITS : CAN_SFF_ID_BITS);

        std::bitset<32> canIdBits{identifier};
        if (mIsExtendedFrame) canIdBits.set(CAN_EXTENDED_BIT_INDEX);
        canIdBits.set(CAN_ERROR_BIT_INDEX);
        canIdBits.set(CAN_RTR_BIT_INDEX);

        mWriteFrame.can_id = canIdBits.to_ullong();
    }

    static std::size_t nearestFittingFdcanFrameSize(std::size_t size) {
        if (size <= 8) return size;
        if (size <= 12) return 12;
        if (size <= 16) return 16;
        if (size <= 20) return 20;
        if (size <= 24) return 24;
        if (size <= 32) return 32;
        if (size <= 48) return 48;
        if (size <= 64) return 64;
        throw std::runtime_error("Too large!");
    }

    void CanNodelet::setFrameData(std::span<const std::byte> data) {
        std::size_t frameSize = nearestFittingFdcanFrameSize(data.size());
        std::memcpy(mWriteFrame.data, data.data(), data.size());
        mWriteFrame.len = frameSize;
    }

    void CanNodelet::canSendRequestCallback(CAN::ConstPtr const& msg) {
        setBus(msg->bus);
        setFrameId(msg->message_id);
        setFrameData({reinterpret_cast<std::byte const*>(msg->data.data()), msg->data.size()});

        writeFrameAsync();
        // printFrame();
    }

    void CanNodelet::printFrame() {
        std::cout << std::format("BUS: {}", mBus) << std::endl;
        std::cout << std::format("CAN_ID: {}", mWriteFrame.can_id) << std::endl;
        std::cout << std::format("LEN: {}", mWriteFrame.len) << std::endl;
        std::cout << "DATA:" << std::endl;
        for (std::uint8_t i = 0; i < mWriteFrame.len; ++i) {
            std::cout << std::format("Index = {}\tData = {:#X}", i, mWriteFrame.data[i]) << std::endl;
        }
    }

    CanNodelet::~CanNodelet() {
        mIoService.stop();
    }

} // namespace mrover

#ifdef MROVER_IS_NODELET

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::CanNodelet, nodelet::Nodelet)

#endif

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_node");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/CanNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}
