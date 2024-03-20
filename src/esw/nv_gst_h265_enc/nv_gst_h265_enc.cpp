#include "nv_gst_h265_enc.hpp"

#if defined(MROVER_IS_JETSON)
constexpr bool IS_JETSON = true;
#else
constexpr bool IS_JETSON = false;
#endif

namespace mrover {

    template<typename T>
    auto gstCheck(T* t) -> T* {
        if (!t) throw std::runtime_error{"Failed to create"};
        return t;
    }

    auto gstCheck(gboolean b) -> void {
        if (!b) throw std::runtime_error{"Failed to create"};
    }

    auto NvGstH265EncNodelet::pullStreamSamplesLoop() -> void {
        // Block until we receive a new encoded chunk from the encoder
        // This is okay since we are on a separate thread
        while (GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(mStreamSink))) {
            // Map the buffer so we can read it, passing it into the stream server which sends it over the network to any clients
            GstBuffer* buffer = gst_sample_get_buffer(sample);
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_READ);
            mStreamServer->feed({reinterpret_cast<std::byte*>(map.data), map.size});

            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
        }
    }

    auto busMessageCallback(GstBus*, GstMessage* message, void* userData) -> gboolean {
        auto encoder = static_cast<NvGstH265EncNodelet*>(userData);
        switch (GST_MESSAGE_TYPE(message)) {
            case GST_MESSAGE_ERROR: {
                GError* error;
                gchar* debug;
                gst_message_parse_error(message, &error, &debug);
                ROS_INFO_STREAM(std::format("Error: {} {}", error->message, debug));
                g_error_free(error);
                g_free(debug);
                g_main_loop_quit(encoder->mMainLoop);
                break;
            }
            case GST_MESSAGE_EOS:
                g_main_loop_quit(encoder->mMainLoop);
                break;
            default:
                break;
        }
        return TRUE;
    }

    auto NvGstH265EncNodelet::initPipeline() -> void {
        ROS_INFO("Initializing and starting GStreamer pipeline...");

        mMainLoop = gstCheck(g_main_loop_new(nullptr, FALSE));

        std::string launch;
        if (mCaptureDevice.empty()) {
            if constexpr (IS_JETSON) {
                // TODO(quintin): Use mCodec
                // ReSharper disable once CppDFAUnreachableCode
                launch = std::format(
                        "appsrc name=imageSource " // App source is pushed to when we get a ROS BGRA image message
                        "! video/x-raw,format=BGRA,width={},height={},framerate=30/1 "
                        "! videoconvert " // Convert BGRA => I420 (YUV) for the encoder, note we are still on the CPU
                        "! video/x-raw,format=I420 "
                        "! nvvidconv " // Upload to GPU memory for the encoder
                        "! video/x-raw(memory:NVMM),format=I420 "
                        "! nvv4l2h265enc bitrate={} iframeinterval=300 vbv-size=33333 insert-sps-pps=true control-rate=constant_bitrate profile=Main num-B-Frames=0 ratecontrol-enable=true preset-level=UltraFastPreset EnableTwopassCBR=false maxperf-enable=true "
                        "! appsink sync=false name=streamSink", // App sink is pulled from (getting H265 chunks) on another thread and sent to the stream server
                        mImageWidth, mImageHeight, mBitrate);
            } else {
                // ReSharper disable once CppDFAUnreachableCode
                launch = std::format(
                        "appsrc name=imageSource "
                        "! video/x-raw,format=BGRA,width={},height={},framerate=30/1 "
                        "! videoconvert "
                        "! nv{}enc "
                        "! appsink name=streamSink",
                        mImageWidth, mImageHeight, mCodec);
            }
        } else {
            assert(IS_JETSON);
            // TODO(quintin): This does not work: https://forums.developer.nvidia.com/t/macrosilicon-usb/157777/4
            //                nvv4l2camerasrc only supports UYUV, but our cameras use I420... NVIDIA is lazy!

            // ReSharper disable once CppDFAUnreachableCode
            launch = std::format(
                    "nvv4l2camerasrc device={} "
                    "! video/x-raw(memory:NVMM),format=UYVY,width={},height={},framerate={}/1 "
                    "! nvvidconv "
                    "! video/x-raw(memory:NVMM),format=I420 "
                    "! nvv4l2{}enc bitrate={} iframeinterval=300 vbv-size=33333 insert-sps-pps=true control-rate=constant_bitrate profile=Main num-B-Frames=0 ratecontrol-enable=true preset-level=UltraFastPreset EnableTwopassCBR=false maxperf-enable=true "
                    "! appsink sync=false name=streamSink",
                    mCaptureDevice, mImageWidth, mImageHeight, mImageFramerate, mCodec, mBitrate);
        }

        ROS_INFO_STREAM(std::format("GStreamer launch: {}", launch));
        mPipeline = gstCheck(gst_parse_launch(launch.c_str(), nullptr));

        if (mCaptureDevice.empty()) mImageSource = gstCheck(gst_bin_get_by_name(GST_BIN(mPipeline), "imageSource"));
        mStreamSink = gstCheck(gst_bin_get_by_name(GST_BIN(mPipeline), "streamSink"));

        GstBus* bus = gst_element_get_bus(mPipeline);
        gst_bus_add_watch(bus, busMessageCallback, this);
        gst_object_unref(bus);

        if (gst_element_set_state(mPipeline, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE)
            throw std::runtime_error{"Failed initial pause on GStreamer pipeline"};

        mMainLoopThread = std::thread{[this] {
            ROS_INFO("Started GStreamer main loop");
            g_main_loop_run(mMainLoop);
            ROS_INFO("Stopped GStreamer main loop");
        }};

        mStreamSinkThread = std::thread{[this] {
            ROS_INFO("Started stream sink thread");
            pullStreamSamplesLoop();
            ROS_INFO("Stopped stream sink thread");
        }};

        ROS_INFO("Initialized and started GStreamer pipeline");
    }

    std::mutex m;

    auto NvGstH265EncNodelet::imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void {
        try {
            std::scoped_lock lock{m};

            if (msg->encoding != sensor_msgs::image_encodings::BGRA8) throw std::runtime_error{"Unsupported encoding"};

            mImageWidth = msg->width;
            mImageHeight = msg->height;

            if (!mPipeline) initPipeline();

            if (!mIsPipelinePlaying) return;

            // "step" is the number of bytes (NOT pixels) in an image row
            std::size_t size = msg->step * msg->height;
            GstBuffer* buffer = gstCheck(gst_buffer_new_allocate(nullptr, size, nullptr));
            GstMapInfo info;
            gst_buffer_map(buffer, &info, GST_MAP_WRITE);
            std::memcpy(info.data, msg->data.data(), size);
            gst_buffer_unmap(buffer, &info);

            if (gst_app_src_push_buffer(GST_APP_SRC(mImageSource), buffer) != GST_FLOW_OK)
                throw std::runtime_error{"Failed to push buffer"};

        } catch (std::exception const& e) {
            ROS_ERROR_STREAM(std::format("Exception encoding frame: {}", e.what()));
            ros::requestShutdown();
        }
    }

    auto NvGstH265EncNodelet::onInit() -> void {
        try {
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();

            mCaptureDevice = mPnh.param<std::string>("device", "");
            mImageTopic = mNh.param<std::string>("image_topic", "image");
            if (!mCaptureDevice.empty()) {
                mImageWidth = mPnh.param<int>("width", 640);
                mImageHeight = mPnh.param<int>("height", 480);
                mImageFramerate = mPnh.param<int>("framerate", 30);
            }
            auto address = mPnh.param<std::string>("address", "0.0.0.0");
            auto port = mPnh.param<int>("port", 8080);
            mBitrate = mPnh.param<int>("bitrate", 2000000);
            mCodec = mPnh.param<std::string>("codec", "h265");

            gst_init(nullptr, nullptr);

            mStreamServer.emplace(
                    address,
                    port,
                    [this] {
                        std::scoped_lock lock{m};
                        ROS_INFO("Client connected");
                        if (gst_element_set_state(mPipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
                            throw std::runtime_error{"Failed to play GStreamer pipeline"};
                        mIsPipelinePlaying = true;
                        ROS_INFO("Playing GStreamer pipeline");
                    },
                    [this] {
                        std::scoped_lock lock{m};
                        ROS_INFO("Client disconnected");
                        gst_app_src_end_of_stream(GST_APP_SRC(mImageSource));
                        if (gst_element_set_state(mPipeline, GST_STATE_READY) == GST_STATE_CHANGE_FAILURE)
                            throw std::runtime_error{"Failed to pause GStreamer pipeline"};
                        std::this_thread::sleep_for(std::chrono::milliseconds{100});
                        mIsPipelinePlaying = false;
                        ROS_INFO("Paused GStreamer pipeline");
                    });

            if (mCaptureDevice.empty()) {
                mImageSubscriber = mNh.subscribe(mImageTopic, 1, &NvGstH265EncNodelet::imageCallback, this);
            } else {
                initPipeline();
            }

        } catch (std::exception const& e) {
            ROS_ERROR_STREAM(std::format("Exception initializing NVIDIA GST H265 Encoder: {}", e.what()));
            ros::requestShutdown();
        }
    }

    NvGstH265EncNodelet::~NvGstH265EncNodelet() {
        if (mImageSource) gst_app_src_end_of_stream(GST_APP_SRC(mImageSource));

        mMainLoopThread.join();
        mStreamSinkThread.join();

        if (mPipeline) {
            gst_element_set_state(mPipeline, GST_STATE_NULL);
            gst_object_unref(mPipeline);
        }
    }

} // namespace mrover