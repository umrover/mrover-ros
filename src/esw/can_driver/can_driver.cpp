#include "can_driver.hpp"

#include <linux/can.h>

#include <boost/asio/read.hpp>
#include <boost/asio/write.hpp>
#include <boost/system/error_code.hpp>

#include <nodelet/loader.h>
#include <ros/init.h>
#include <ros/names.h>
#include <ros/this_node.h>

namespace mrover {

    static int checkSyscallResult(int result) {
        if (result < 0) throw std::system_error{errno, std::generic_category()};

        return result;
    }

    static void checkErrorCode(boost::system::error_code const& ec) {
        if (ec.value() == boost::system::errc::success) return;

        throw std::runtime_error(std::format("Boost failure: {} {}", ec.value(), ec.message()));
    }

    static std::uint8_t nearestFittingFdcanFrameSize(std::size_t size) {
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

    void CanNodelet::onInit() {
        try {
            NODELET_INFO("CAN Node starting...");

            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();

            mInterface = mPnh.param<std::string>("interface", "can0");
            mIsExtendedFrame = mPnh.param<bool>("is_extended_frame", true);
            mBitrate = static_cast<std::uint32_t>(mPnh.param<int>("bitrate", 500000));
            mBitratePrescaler = static_cast<std::uint32_t>(mPnh.param<int>("bitrate_prescaler", 2));

            XmlRpc::XmlRpcValue can_devices;
            mNh.getParam("can/devices", can_devices);

            for (auto& can_device: can_devices) {
                assert(can_device.hasMember("name") &&
                       can_device["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
                std::string device_name = static_cast<std::string>(can_device["name"]);
                mCanSubscriberByDeviceName[device_name] = mNh.subscribe<CAN>(std::format("can/{}/out", device_name), 16, &CanNodelet::frameSendRequestCallback, this);
                mCanPublisherByDeviceName[device_name] = mNh.advertise<CAN>(std::format("can/{}/in", device_name), 16);

                assert(can_device.hasMember("id") &&
                       can_device["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                auto can_id = (uint8_t) static_cast<int>(can_device["id"]);

                mDeviceNameByID[can_id] = device_name;
                mIDByDeviceName[device_name] = can_id;

                assert(can_device.hasMember("bus") &&
                       can_device["bus"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                auto can_bus = (uint8_t) static_cast<int>(can_device["bus"]);

                mBusByDeviceName[device_name] = bus;
            }

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

                    frameReadCallback();

                    // Ready for the next frame, start listening again
                    // Note this is recursive, but it is safe because it is async
                    // i.e. the stack is not growing
                    readFrameAsync();
                });
    }

    void CanNodelet::frameReadCallback() { // NOLINT(*-no-recursion)
        CAN msg;
        // uint8_t bus = mBusByDeviceName[source]; // TODO - maybe ignore bus. if all ids are unique, then its ok to ignore.
        // TODO - parse this: std::bit_cast<RawCanFdId>(mReadFrame.can_id).identifier
        uint8_t raw_source_id = 0;      // TODO - fix
        uint8_t raw_destination_id = 0; // TODO - fix
        msg.source = mDeviceNameByID[raw_source_id];
        msg.destination = mDeviceNameByID[raw_destination_id];
        msg.data.assign(mReadFrame.data, mReadFrame.data + mReadFrame.len);

        ROS_DEBUG_STREAM("Received CAN message:\n"
                         << msg);

        mCanPublisherByDeviceName[source].publish(msg);
    }

    void CanNodelet::frameSendRequestCallback(CAN::ConstPtr const& msg) {
        ROS_DEBUG_STREAM("Received request to send CAN message:\n"
                         << *msg);

        // Check that the identifier is not too large
        // TODO - see if following assert is still needed
        // assert(std::bit_width(msg->message_id) <= mIsExtendedFrame ? CAN_EFF_ID_BITS : CAN_SFF_ID_BITS);

        // TODO - need to send it on the proper bus.
        // right now idek what bus it sends from (sincerely, guthrie)
        uint8_t bus = mBusByDeviceName[msg->destination]; // TODO(owen) support multiple buses

        // need to create CAN message ID
        uint16_t message_id;
        CanFdMessageId can_message_id;
        can_message_id.destination = mIDByDeviceName[msg->destination];
        can_message_id.source = mIDByDeviceName[msg->source];
        can_message_id.reply_required = 0; // TODO - i don't think we currently care about this right now

        // Craft the SocketCAN frame from the ROS message
        canfd_frame frame{
                .can_id = std::bit_cast<canid_t>(RawCanFdId{
                        .identifier = can_message_id,
                        .isExtendedFrame = mIsExtendedFrame,
                }),
                .len = nearestFittingFdcanFrameSize(msg->data.size()),
        };
        std::memcpy(frame.data, msg->data.data(), msg->data.size());

        std::size_t written = boost::asio::write(mStream.value(), boost::asio::buffer(std::addressof(frame), sizeof(frame)));
        if (written != sizeof(frame)) throw std::runtime_error("Failed to write entire CAN frame");

        ROS_DEBUG_STREAM("Sent CAN message");
    }

    CanNodelet::~CanNodelet() {
        mIoService.stop(); // This causes the io thread to finish
    }

} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_node");

    // Start the CAN Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/CanNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::CanNodelet, nodelet::Nodelet)
#endif
