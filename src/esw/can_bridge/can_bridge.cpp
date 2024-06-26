#include "can_bridge.hpp"

namespace mrover {

    static auto checkSyscallResult(int result) -> int {
        if (result < 0) throw std::system_error{errno, std::generic_category()};

        return result;
    }

    static auto checkErrorCode(boost::system::error_code const& ec) -> void {
        if (ec.value() == boost::system::errc::success) return;

        throw std::runtime_error(std::format("Boost failure: {} {}", ec.value(), ec.message()));
    }

    static auto nearestFittingFdcanFrameSize(std::size_t size) -> std::uint8_t {
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

    auto CanNodelet::onInit() -> void {
        try {
            NODELET_INFO("CAN Node starting...");

            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();

            mInterface = mPnh.param<std::string>("interface", "can0");
            mIsExtendedFrame = mPnh.param<bool>("is_extended_frame", true);

            XmlRpc::XmlRpcValue canDevices;
            mNh.getParam("can/devices", canDevices);
            if (canDevices.getType() == XmlRpc::XmlRpcValue::TypeArray) {
                for (int size = canDevices.size(), i = 0; i < size; ++i) {
                    XmlRpc::XmlRpcValue const& canDevice = canDevices[i];

                    auto bus = xmlRpcValueToTypeOrDefault<std::uint8_t>(canDevice, "bus");

                    mBus = mInterface.back() - '0';
                    if (bus != mBus) continue;

                    assert(canDevice.getType() == XmlRpc::XmlRpcValue::TypeStruct);

                    auto name = xmlRpcValueToTypeOrDefault<std::string>(canDevice, "name");

                    auto id = xmlRpcValueToTypeOrDefault<std::uint8_t>(canDevice, "id");

                    mDevices.emplace(name,
                                     CanFdAddress{
                                             .bus = bus,
                                             .id = id,
                                     });

                    mDevicesPubSub.emplace(name,
                                           CanFdPubSub{
                                                   .publisher = mNh.advertise<CAN>(std::format("can/{}/in", name), 16),
                                                   .subscriber = mNh.subscribe<CAN>(std::format("can/{}/out", name), 16, &CanNodelet::frameSendRequestCallback, this),
                                           });

                    NODELET_DEBUG_STREAM(std::format("Added CAN device: {} (bus: {}, id: {})", name, bus, id));
                }
            } else {
                NODELET_WARN("No CAN devices specified or config was invalid. Did you forget to load the correct ROS parameters?");
                NODELET_WARN("For example before testing the devboard run: \"rosparam load config/esw.yml\"");
            }

            mCanNetLink = CanNetLink{mInterface};

            int socketFileDescriptor = setupSocket();
            mStream.emplace(mIoService);
            mStream->assign(socketFileDescriptor);

            readFrameAsync();

            // Since "onInit" needs to return, kick off a self-joining thread to run the IO concurrently
            mIoThread = std::jthread{[this] { mIoService.run(); }};

            NODELET_INFO("CAN bridge started");

        } catch (std::exception const& exception) {
            NODELET_FATAL_STREAM(std::format("CAN bridge failed to start: {}", exception.what()));
            ros::shutdown();
        }
    }

    auto CanNodelet::setupSocket() const -> int {
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

    auto CanNodelet::readFrameAsync() -> void { // NOLINT(*-no-recursion)
        // You would think we would have to read the header first to find the data length (which is not always 64 bytes) and THEN read the data
        // However socketcan is nice and just requires we read the max length
        // It then puts the actual length in the header
        async_read(
                mStream.value(),
                boost::asio::buffer(&mReadFrame, sizeof(mReadFrame)),
                // Supply lambda that is called on completion
                [this](boost::system::error_code const& ec, [[maybe_unused]] std::size_t bytes) { // NOLINT(*-no-recursion)
                    checkErrorCode(ec);
                    assert(bytes == sizeof(mReadFrame));

                    frameReadCallback();

                    // Ready for the next frame, start listening again
                    // Note this is recursive, but it is safe because it is async
                    // i.e. the stack is not growing
                    readFrameAsync();
                });
    }

    auto CanNodelet::frameReadCallback() -> void { // NOLINT(*-no-recursion)
        auto [identifier, isErrorFrame, isRemoteTransmissionRequest, isExtendedFrame] = std::bit_cast<RawCanFdId>(mReadFrame.can_id);
        auto [destination, source, isReplyRequired] = std::bit_cast<CanFdMessageId>(static_cast<std::uint16_t>(identifier));

        optional_ref<std::string> sourceDeviceName = mDevices.backward(CanFdAddress{
                .bus = mBus,
                .id = source,
        });
        if (!sourceDeviceName) {
            NODELET_WARN_STREAM(std::format("Received CAN message on interface {} that had an unknown source ID: {}", mInterface, std::uint8_t{source}));
            return;
        }

        optional_ref<std::string> destinationDeviceName = mDevices.backward(CanFdAddress{
                .bus = mBus,
                .id = destination,
        });
        if (!destinationDeviceName) {
            NODELET_WARN_STREAM(std::format("Received CAN message on interface {} that had an unknown destination ID: {}", mInterface, std::uint8_t{destination}));
            return;
        }

        CAN msg;
        msg.source = sourceDeviceName.value();
        msg.destination = destinationDeviceName.value();
        msg.data.assign(mReadFrame.data, mReadFrame.data + mReadFrame.len);

        mDevicesPubSub.at(sourceDeviceName.value()).publisher.publish(msg);
    }

    auto CanNodelet::frameSendRequestCallback(CAN::ConstPtr const& msg) -> void {
        optional_ref<CanFdAddress> source = mDevices.forward(msg->source);
        if (!source) {
            NODELET_WARN_STREAM(std::format("Sending CAN message on interface {} that had an unknown source: {}", mInterface, msg->source));
            return;
        }

        optional_ref<CanFdAddress> destination = mDevices.forward(msg->destination);
        if (!destination) {
            NODELET_WARN_STREAM(std::format("Sending CAN message on interface {} that had an unknown destination: {}", mInterface, msg->destination));
            return;
        }

        CanFdMessageId messageId{
                .destination = destination->get().id,
                .source = source->get().id,
                .replyRequired = static_cast<bool>(msg->reply_required),
        };

        // Craft the SocketCAN frame from the ROS message
        canfd_frame frame{
                .can_id = std::bit_cast<canid_t>(RawCanFdId{
                        .identifier = std::bit_cast<std::uint16_t>(messageId),
                        .isExtendedFrame = mIsExtendedFrame,
                }),
                .len = nearestFittingFdcanFrameSize(msg->data.size()),
        };
        std::memcpy(frame.data, msg->data.data(), msg->data.size());

        try {
            if (std::size_t written = boost::asio::write(mStream.value(), boost::asio::buffer(std::addressof(frame), sizeof(frame)));
                written != sizeof(frame)) {
                NODELET_FATAL_STREAM(std::format("Failed to write CAN frame to socket! Expected to write {} bytes, but only wrote {} bytes", sizeof(frame), written));
                ros::shutdown();
            }
        } catch (boost::system::system_error const& error) {
            // check if ran out of buffer space
            if (error.code() == boost::asio::error::no_buffer_space) {
                NODELET_WARN_STREAM_THROTTLE(1, "No buffer space available to send CAN message. This usually indicates an electrical problem with the bus. CAN will avoid sending out messages if it can not see other devices.");
                return;
            }
            ros::shutdown();
        }
    }

    CanNodelet::~CanNodelet() {
        mIoService.stop(); // This causes the io thread to finish
    }

} // namespace mrover
