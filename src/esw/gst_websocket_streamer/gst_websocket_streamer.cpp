#include "gst_websocket_streamer.hpp"

#if defined(MROVER_IS_JETSON)
constexpr bool IS_JETSON = true;
#else
constexpr bool IS_JETSON = false;
#endif

namespace mrover {

    using namespace std::string_view_literals;

    template<typename T>
    auto gstCheck(T* t) -> T* {
        if (!t) throw std::runtime_error{"Failed to create"};
        return t;
    }

    auto gstCheck(gboolean b) -> void {
        if (!b) throw std::runtime_error{"Failed to create"};
    }

    auto GstWebsocketStreamerNodelet::pullStreamSamplesLoop() -> void {
        // Block until we receive a new encoded chunk from the encoder
        // This is okay since we are on a separate thread
        while (GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(mStreamSink))) {
            // Map the buffer so we can read it, passing it into the stream server which sends it over the network to any clients
            GstBuffer* buffer = gst_sample_get_buffer(sample);
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_READ);
            mStreamServer->broadcast({reinterpret_cast<std::byte*>(map.data), map.size});

            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
        }
    }

    auto GstWebsocketStreamerNodelet::initPipeline() -> void {
        ROS_INFO("Initializing and starting GStreamer pipeline...");

        mMainLoop = gstCheck(g_main_loop_new(nullptr, FALSE));

        std::string launch;
        if (mDeviceNode.empty()) {
            if constexpr (IS_JETSON) {
                // ReSharper disable once CppDFAUnreachableCode
                launch = std::format(
                        // App source is pushed to when we get a ROS BGRA image message
                        // is-live prevents frames from being pushed when the pipeline is in READY
                        "appsrc name=imageSource is-live=true "
                        "! video/x-raw,format=BGRA,width={},height={},framerate=30/1 "
                        "! videoconvert " // Convert BGRA => I420 (YUV) for the encoder, note we are still on the CPU
                        "! video/x-raw,format=I420 "
                        "! nvvidconv " // Upload to GPU memory for the encoder
                        "! video/x-raw(memory:NVMM),format=I420 "
                        "! nvv4l2h265enc name=encoder bitrate={} iframeinterval=300 vbv-size=33333 insert-sps-pps=true control-rate=constant_bitrate profile=Main num-B-Frames=0 ratecontrol-enable=true preset-level=UltraFastPreset EnableTwopassCBR=false maxperf-enable=true "
                        // App sink is pulled from (getting H265 chunks) on another thread and sent to the stream server
                        // sync=false is needed to avoid weirdness, going from playing => ready => playing will not work otherwise
                        "! appsink name=streamSink sync=false",
                        mImageWidth, mImageHeight, mBitrate);
            } else {
                // ReSharper disable once CppDFAUnreachableCode
                launch = std::format(
                        "appsrc name=imageSource is-live=true "
                        "! video/x-raw,format=BGRA,width={},height={},framerate=30/1 "
                        "! nvh265enc name=encoder "
                        "! appsink name=streamSink sync=false",
                        mImageWidth, mImageHeight);
            }
        } else {
            if constexpr (IS_JETSON) {
                // ReSharper disable once CppDFAUnreachableCode

                // TODO(quintin): I had to apply this patch: https://forums.developer.nvidia.com/t/macrosilicon-usb/157777/4
                //                nvv4l2camerasrc only supports UYUV by default, but our cameras are YUY2 (YUYV)

                launch = std::format(
                        // "nvv4l2camerasrc device={} "

                        "v4l2src device={} "

                        // "! video/x-raw(memory:NVMM),format=YUY2,width={},height={},framerate={}/1 "
                        "! image/jpeg,width={},height={},framerate={}/1 "
                        "! nvv4l2decoder mjpeg=1 "

                        "! nvvidconv "
                        "! video/x-raw(memory:NVMM),format=NV12 "
                        "! nvv4l2h265enc name=encoder bitrate={} iframeinterval=300 vbv-size=33333 insert-sps-pps=true control-rate=constant_bitrate profile=Main num-B-Frames=0 ratecontrol-enable=true preset-level=UltraFastPreset EnableTwopassCBR=false maxperf-enable=true "
                        "! appsink name=streamSink sync=false",
                        mDeviceNode, mImageWidth, mImageHeight, mImageFramerate, mBitrate);
            } else {
                // ReSharper disable once CppDFAUnreachableCode
                launch = std::format(
                        "v4l2src device={}  "
                        "! video/x-raw,format=YUY2,width={},height={},framerate=30/1 "
                        "! videoconvert "
                        "! nvh265enc name=encoder "
                        "! appsink name=streamSink sync=false",
                        mDeviceNode, mImageWidth, mImageHeight);
            }
        }

        ROS_INFO_STREAM(std::format("GStreamer launch: {}", launch));
        mPipeline = gstCheck(gst_parse_launch(launch.c_str(), nullptr));

        if (mDeviceNode.empty()) mImageSource = gstCheck(gst_bin_get_by_name(GST_BIN(mPipeline), "imageSource"));
        mStreamSink = gstCheck(gst_bin_get_by_name(GST_BIN(mPipeline), "streamSink"));

        if (gst_element_set_state(mPipeline, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE)
            throw std::runtime_error{"Failed initial pause on GStreamer pipeline"};

        mMainLoopThread = std::thread{[this] {
            ROS_INFO("Started GStreamer main loop");
            g_main_loop_run(mMainLoop);
            std::cout << "Stopped GStreamer main loop" << std::endl;
        }};

        mStreamSinkThread = std::thread{[this] {
            ROS_INFO("Started stream sink thread");
            pullStreamSamplesLoop();
            std::cout << "Stopped stream sink thread" << std::endl;
        }};

        ROS_INFO("Initialized and started GStreamer pipeline");
    }

    auto GstWebsocketStreamerNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void {
        try {
            if (mStreamServer->clientCount() == 0) return;

            if (msg->encoding != sensor_msgs::image_encodings::BGRA8) throw std::runtime_error{"Unsupported encoding"};

            cv::Size receivedSize{static_cast<int>(msg->width), static_cast<int>(msg->height)};
            cv::Mat bgraFrame{receivedSize, CV_8UC4, const_cast<std::uint8_t*>(msg->data.data()), msg->step};

            if (cv::Size targetSize{static_cast<int>(mImageWidth), static_cast<int>(mImageHeight)};
                receivedSize != targetSize) {
                ROS_WARN_ONCE("Image size does not match pipeline app source size, will resize");
                resize(bgraFrame, bgraFrame, targetSize);
            }

            // "step" is the number of bytes (NOT pixels) in an image row
            std::size_t size = bgraFrame.step * bgraFrame.rows;
            GstBuffer* buffer = gstCheck(gst_buffer_new_allocate(nullptr, size, nullptr));
            GstMapInfo info;
            gst_buffer_map(buffer, &info, GST_MAP_WRITE);
            std::memcpy(info.data, bgraFrame.data, size);
            gst_buffer_unmap(buffer, &info);

            gst_app_src_push_buffer(GST_APP_SRC(mImageSource), buffer);

        } catch (std::exception const& e) {
            ROS_ERROR_STREAM(std::format("Exception encoding frame: {}", e.what()));
            ros::requestShutdown();
        }
    }

    auto GstWebsocketStreamerNodelet::onInit() -> void {
        try {
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();

            mDeviceNode = mPnh.param<std::string>("dev_node", "");
            mDevicePath = mPnh.param<std::string>("dev_path", "1");
            mImageTopic = mPnh.param<std::string>("image_topic", "image");
            mImageWidth = mPnh.param<int>("width", 640);
            mImageHeight = mPnh.param<int>("height", 480);
            mImageFramerate = mPnh.param<int>("framerate", 30);
            auto address = mPnh.param<std::string>("address", "0.0.0.0");
            auto port = mPnh.param<int>("port", 8081);
            mBitrate = mPnh.param<int>("bitrate", 2000000);

            gst_init(nullptr, nullptr);

            if (!mDevicePath.empty()) {
                // libusb_context* context{};
                // if (libusb_init(&context) != 0) throw std::runtime_error{"Failed to initialize libusb"};
                //
                // libusb_device** list{};
                // ssize_t count = libusb_get_device_list(context, &list);
                // if (count < 0) throw std::runtime_error{"Failed to get device list"};
                //
                // // Find the libusb device associated with our camera
                //
                // auto it = std::ranges::find_if(list, list + count, [this](libusb_device* device) {
                //     libusb_device_descriptor descriptor{};
                //     if (libusb_get_device_descriptor(device, &descriptor) != 0) return false;
                //     return std::format("{}-{}", libusb_get_bus_number(device), libusb_get_port_number(device)) == mUsbIdentifier;
                // });
                // if (it == list + count) throw std::runtime_error{"Failed to find USB device"};
                // libusb_device* targetUsbDevice = *it;

                udev* udevContext = udev_new();
                if (!udevContext) throw std::runtime_error{"Failed to initialize udev"};

                udev_enumerate* enumerate = udev_enumerate_new(udevContext);
                if (!enumerate) throw std::runtime_error{"Failed to create udev enumeration"};

                udev_enumerate_add_match_subsystem(enumerate, "video4linux");
                udev_enumerate_scan_devices(enumerate);

                udev_list_entry* devices = udev_enumerate_get_list_entry(enumerate);
                if (!devices) throw std::runtime_error{"Failed to get udev device list"};

                udev_device* device{};
                udev_list_entry* entry;
                udev_list_entry_foreach(entry, devices) {
                    device = udev_device_new_from_syspath(udevContext, udev_list_entry_get_name(entry));

                    if (udev_device_get_devpath(device) != mDevicePath) continue;

                    if (!device) throw std::runtime_error{"Failed to get udev device"};

                    break;

                    // for (udev_list_entry* listEntry = udev_device_get_properties_list_entry(device); listEntry != nullptr; listEntry = udev_list_entry_get_next(listEntry)) {
                    //     char const* name = udev_list_entry_get_name(listEntry);
                    //     char const* value = udev_list_entry_get_value(listEntry);
                    //     ROS_INFO_STREAM(std::format("udev: {} = {}", name, value));
                    // }

                    // if (std::stoi(udev_device_get_sysattr_value(device, "busnum")) == libusb_get_bus_number(targetUsbDevice) &&
                    //     std::stoi(udev_device_get_sysattr_value(device, "devnum")) == libusb_get_device_address(targetUsbDevice)) {
                    //     targetUdevDevice = device;
                    //     break;
                    // }
                }
                if (!device) throw std::runtime_error{"Failed to find udev device"};

                mDeviceNode = udev_device_get_devnode(device);
            }

            initPipeline();

            mStreamServer.emplace(
                    address,
                    port,
                    // Note that the encodings we use send an initial seed frame (I-frame) and then encode subsequent frames based on motion deltas
                    // So when a new client connects, we need to restart the pipeline to ensure they get an I-frame

                    // When restarting we want to set the pipeline state to ready instead of paused or null
                    // Paused will not re-generate I-frames
                    // Null tears down too much and would require a full reinitialization
                    [this] {
                        ROS_INFO("Client connected");
                        if (mStreamServer->clientCount() > 1)
                            // Ensure new clients get an I-frame as their first frame
                            if (gst_element_set_state(mPipeline, GST_STATE_READY) == GST_STATE_CHANGE_FAILURE)
                                throw std::runtime_error{"Failed to play GStreamer pipeline"};
                        if (gst_element_set_state(mPipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
                            throw std::runtime_error{"Failed to play GStreamer pipeline"};

                        ROS_INFO("Playing GStreamer pipeline");
                    },
                    [this] {
                        ROS_INFO("Client disconnected");
                        // Stop the pipeline only if there are no more clients
                        if (mStreamServer->clientCount() == 0)
                            if (gst_element_set_state(mPipeline, GST_STATE_READY) == GST_STATE_CHANGE_FAILURE)
                                throw std::runtime_error{"Failed to pause GStreamer pipeline"};

                        ROS_INFO("Stopped GStreamer pipeline");
                    });

            if (mDeviceNode.empty()) {
                mImageSubscriber = mNh.subscribe(mImageTopic, 1, &GstWebsocketStreamerNodelet::imageCallback, this);
            }

        } catch (std::exception const& e) {
            ROS_ERROR_STREAM(std::format("Exception initializing NVIDIA GST H265 Encoder: {}", e.what()));
            ros::requestShutdown();
        }
    }

    GstWebsocketStreamerNodelet::~GstWebsocketStreamerNodelet() {
        if (mImageSource) gst_app_src_end_of_stream(GST_APP_SRC(mImageSource));

        if (mMainLoop) {
            g_main_loop_quit(mMainLoop);
            mMainLoopThread.join();
            g_main_loop_unref(mMainLoop);
        }

        if (mPipeline) {
            gst_element_set_state(mPipeline, GST_STATE_NULL);
            mStreamSinkThread.join();
            gst_object_unref(mPipeline);
        }

        mStreamServer.reset();
    }

} // namespace mrover