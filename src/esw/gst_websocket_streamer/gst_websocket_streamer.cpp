#include "gst_websocket_streamer.hpp"

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

            // Prefix the encoded chunk with metadata for the decoder on the browser side
            std::vector<std::byte> chunk(sizeof(ChunkHeader) + map.size);
            std::memcpy(chunk.data(), &mChunkHeader, sizeof(ChunkHeader));
            std::memcpy(chunk.data() + sizeof(ChunkHeader), map.data, map.size);

            mStreamServer->broadcast(chunk);

            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
        }
    }

    auto GstWebsocketStreamerNodelet::initPipeline() -> void {
        ROS_INFO("Initializing and starting GStreamer pipeline...");

        mMainLoop = gstCheck(g_main_loop_new(nullptr, FALSE));

        // Source
        std::string launch;
        if (mDeviceNode.empty()) {
            // App source is pushed to when we get a ROS BGRA image message
            // is-live prevents frames from being pushed when the pipeline is in READY
            launch += "appsrc name=imageSource is-live=true ";
        } else {
            launch += std::format("v4l2src device={} ", mDeviceNode);
        }
        // Source format
        std::string captureFormat = mDeviceNode.empty() ? "video/x-raw,format=BGRA" : mDecodeJpegFromDevice ? "image/jpeg"
                                                                                                            : "video/x-raw,format=YUY2";
        launch += std::format("! {},width={},height={},framerate={}/1 ",
                              captureFormat, mImageWidth, mImageHeight, mImageFramerate);
        // Source decoder and H265 encoder
        if (gst_element_factory_find("nvv4l2h265enc")) {
            // Most likely on the Jetson
            if (captureFormat.contains("jpeg")) {
                // Mostly used with USB cameras, MPEG capture uses way less USB bandwidth
                launch +=
                        // TODO(quintin): I had to apply this patch: https://forums.developer.nvidia.com/t/macrosilicon-usb/157777/4
                        //                nvv4l2camerasrc only supports UYUV by default, but our cameras are YUY2 (YUYV)
                        // "nvv4l2camerasrc device={} "
                        // "! video/x-raw(memory:NVMM),format=YUY2,width={},height={},framerate={}/1 "

                        "! nvv4l2decoder mjpeg=1 " // Hardware-accelerated JPEG decoding, output is apparently some unknown proprietary format
                        "! nvvidconv "             // Convert from proprietary format to NV12 so the encoder understands it
                        "! video/x-raw(memory:NVMM),format=NV12 ";
            } else {
                launch += "! videoconvert " // Convert to I420 for the encoder, note we are still on the CPU
                          "! video/x-raw,format=I420 "
                          "! nvvidconv " // Upload to GPU memory for the encoder
                          "! video/x-raw(memory:NVMM),format=I420 ";
            }
            launch += std::format("! nvv4l2h265enc name=encoder bitrate={} iframeinterval=300 vbv-size=33333 insert-sps-pps=true control-rate=constant_bitrate profile=Main num-B-Frames=0 ratecontrol-enable=true preset-level=UltraFastPreset EnableTwopassCBR=false maxperf-enable=true ",
                                  mBitrate);
            mChunkHeader.codec = ChunkHeader::Codec::H265;
        } else if (gst_element_factory_find("nvh265enc")) {
            // For desktop/laptops with the custom NVIDIA bad gstreamer plugins built (a massive pain to do!)
            if (captureFormat.contains("jpeg"))
                launch += "! jpegdec ";
            else
                launch += "! videoconvert ";
            launch += "! nvh265enc name=encoder ";
            mChunkHeader.codec = ChunkHeader::Codec::H265;
        } else {
            // For desktop/laptops with no hardware encoder
            if (captureFormat.contains("jpeg"))
                launch += "! jpegdec ";
            else
                launch += "! videoconvert ";
            launch += std::format("! x264enc tune=zerolatency bitrate={} name=encoder ", mBitrate);
            mChunkHeader.codec = ChunkHeader::Codec::H264;
        }
        // App sink is pulled from (getting H265 chunks) on another thread and sent to the stream server
        // sync=false is needed to avoid weirdness, going from playing => ready => playing will not work otherwise
        launch += "! appsink name=streamSink sync=false";

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

    auto widthAndHeightToResolution(std::uint32_t width, std::uint32_t height) -> ChunkHeader::Resolution {
        if (width == 640 && height == 480) return ChunkHeader::Resolution::EGA;
        if (width == 1280 && height == 720) return ChunkHeader::Resolution::HD;
        if (width == 1920 && height == 1080) return ChunkHeader::Resolution::FHD;
        throw std::runtime_error{"Unsupported resolution"};
    }

    auto findDeviceNode(std::string_view devicePath) -> std::string {
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

            if (udev_device_get_devpath(device) != devicePath) continue;

            if (!device) throw std::runtime_error{"Failed to get udev device"};

            break;
        }
        if (!device) throw std::runtime_error{"Failed to find udev device"};

        std::string deviceNode = udev_device_get_devnode(device);

        udev_device_unref(device);
        udev_enumerate_unref(enumerate);
        udev_unref(udevContext);

        return deviceNode;
    }

    auto GstWebsocketStreamerNodelet::onInit() -> void {
        try {
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();

            mDeviceNode = mPnh.param<std::string>("dev_node", "");
            mDevicePath = mPnh.param<std::string>("dev_path", "1");
            mDecodeJpegFromDevice = mPnh.param<bool>("decode_jpeg_from_device", true);
            mImageTopic = mPnh.param<std::string>("image_topic", "image");
            mImageWidth = mPnh.param<int>("width", 640);
            mImageHeight = mPnh.param<int>("height", 480);
            mImageFramerate = mPnh.param<int>("framerate", 30);
            auto address = mPnh.param<std::string>("address", "0.0.0.0");
            auto port = mPnh.param<int>("port", 8081);
            mBitrate = mPnh.param<int>("bitrate", 2000000);

            mChunkHeader.resolution = widthAndHeightToResolution(mImageWidth, mImageHeight);

            gst_init(nullptr, nullptr);

            if (!mDevicePath.empty()) mDeviceNode = findDeviceNode(mDevicePath);

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
            ROS_ERROR_STREAM(std::format("Exception initializing gstreamer websocket streamer: {}", e.what()));
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