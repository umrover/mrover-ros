#include "can_node.hpp"

#include <linux/can.h>

#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/system/error_code.hpp>

#include <nodelet/loader.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/this_node.h>

namespace mrover {

    static void printFrame(canfd_frame const& frame) {
        std::ostringstream stream;
        auto id = std::bit_cast<FdcanId>(frame.can_id);
        stream << std::format("CANFD ID: {:#X}\n", std::uint32_t{id.identifier});
        stream << std::format("LENGTH: {}\n", frame.len);
        stream << "DATA:\n";
        for (std::uint8_t i = 0; i < frame.len; ++i) {
            stream << std::format("Index = {}\tData = {:#X}\n", i, frame.data[i]);
        }
        ROS_INFO_STREAM(stream.str());
    }

    static void checkErrorCode(boost::system::error_code const& ec) {
        if (ec.value() == boost::system::errc::success) return;

        throw std::runtime_error(std::format("Boost failure: {} {}", ec.value(), ec.message()));
    }

    static int checkSyscallResult(int result) {
        if (result < 0) throw std::system_error{errno, std::generic_category()};

        return result;
    }

    void CanNodelet::onInit() {
        try {
            NODELET_INFO("CAN Node starting...");

            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();

            mInterface = mPnh.param<std::string>("interface", "can0");
            mIsVerbose = mPnh.param<bool>("verbose", true);
            mIsExtendedFrame = mPnh.param<bool>("is_extended_frame", true);
            mBitrate = static_cast<std::uint32_t>(mPnh.param<int>("bitrate", 500000));
            mBitratePrescaler = static_cast<std::uint32_t>(mPnh.param<int>("bitrate_prescaler", 2));

            mCanSubscriber = mNh.subscribe<CAN>("can_requests", 16, &CanNodelet::canSendRequestCallback, this);
            mCanPublisher = mNh.advertise<CAN>("can_data", 16);

            mCanNetLink = {mInterface, mBitrate, mBitratePrescaler};

            int socketFileDescriptor = setupSocket();
            mStream.emplace(mIoService);
            mStream->assign(socketFileDescriptor);

            readFrameAsync();

            // Since "onInit" needs to return, kick off a self-joining thread to run the IO concurrently
            mIoThread = std::jthread{[this] { mIoService.run(); }};

            NODELET_INFO("CAN Node started");

        } catch (std::exception const& exception) {
            NODELET_ERROR_STREAM("Exception in initialization: " << exception.what());
            ros::shutdown();
        }
    }

    int CanNodelet::setupSocket() {
        int socketFd = checkSyscallResult(socket(PF_CAN, SOCK_RAW, CAN_RAW));
        NODELET_INFO_STREAM("Opened CAN socket with file descriptor: " << socketFd);

        ifreq ifr{};
        std::strcpy(ifr.ifr_name, mInterface.c_str());
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
        // You would think we would have to read the header first to find the data length (which is not always 64 bytes) and THEN read the data
        // However socketcan is nice and just requires we read the max length
        // It then puts the actual length in the header
        boost::asio::async_read(
                mStream.value(),
                boost::asio::buffer(&mReadFrame, sizeof(mReadFrame)),
                // Supply lambda that is called on completion
                [this](boost::system::error_code const& ec, std::size_t bytes) { // NOLINT(*-no-recursion)
                    checkErrorCode(ec);
                    assert(bytes == sizeof(mReadFrame));

                    processReadFrame();

                    // Ready for the next frame, start listening again
                    // Note this is recursive, but it is safe because it is async
                    // i.e. the stack is not growing
                    readFrameAsync();
                });
    }

    void CanNodelet::writeFrameAsync() {
        boost::asio::async_write(
                mStream.value(),
                boost::asio::buffer(&mWriteFrame, sizeof(mWriteFrame)),
                [](boost::system::error_code const& ec, std::size_t bytes) {
                    checkErrorCode(ec);
                    assert(bytes == sizeof(mWriteFrame));
                });
    }

    void CanNodelet::processReadFrame() { // NOLINT(*-no-recursion)
        if (mIsVerbose) printFrame(mReadFrame);

        CAN msg;
        msg.bus = 0;
        msg.message_id = std::bit_cast<FdcanId>(mReadFrame.can_id).identifier;
        msg.data.assign(mReadFrame.data, mReadFrame.data + mReadFrame.len);

        mCanPublisher.publish(msg);
    }

    void CanNodelet::setBus(std::uint8_t bus) {
        this->mBus = bus;
    }

    void CanNodelet::setFrameId(std::uint32_t identifier) {
        // Check that the identifier is not too large
        assert(std::bit_width(identifier) <= mIsExtendedFrame ? CAN_EFF_ID_BITS : CAN_SFF_ID_BITS);

        mWriteFrame.can_id = std::bit_cast<std::uint32_t>(FdcanId{
                .identifier = identifier,
                .isExtendedFrame = mIsExtendedFrame,
        });
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
        mWriteFrame.len = frameSize;
        std::memcpy(mWriteFrame.data, data.data(), data.size());
    }

    void CanNodelet::canSendRequestCallback(CAN::ConstPtr const& msg) {
        setBus(msg->bus);
        setFrameId(msg->message_id);
        setFrameData({reinterpret_cast<std::byte const*>(msg->data.data()), msg->data.size()});

        writeFrameAsync();

        if (mIsVerbose) printFrame(mWriteFrame);
    }

    CanNodelet::~CanNodelet() {
        mIoService.stop();
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

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::CanNodelet, nodelet::Nodelet)
#endif
