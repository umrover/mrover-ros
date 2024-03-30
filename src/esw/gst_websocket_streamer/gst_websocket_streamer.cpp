#include "gst_websocket_streamer.hpp"

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
        if (mCaptureDevice.empty()) {
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
                        "! videoconvert "
                        "! nvh265enc name=encoder "
                        "! appsink name=streamSink sync=false",
                        mImageWidth, mImageHeight);
            }
        } else {
            if constexpr (IS_JETSON) {
                // TODO(quintin): I had to apply this patch: https://forums.developer.nvidia.com/t/macrosilicon-usb/157777/4
                //                nvv4l2camerasrc only supports UYUV by default, but our cameras are YUY2 (YUYV)
                // ReSharper disable once CppDFAUnreachableCode

                // launch = std::format(
                //         "nvv4l2camerasrc device={} "
                //         "! video/x-raw(memory:NVMM),format=YUY2,width={},height={},framerate={}/1 "
                //         "! nvvidconv "
                //         "! video/x-raw(memory:NVMM),format=I420 "
                //         "! nvv4l2h265enc name=encoder bitrate={} iframeinterval=300 vbv-size=33333 insert-sps-pps=true control-rate=constant_bitrate profile=Main num-B-Frames=0 ratecontrol-enable=true preset-level=UltraFastPreset EnableTwopassCBR=false maxperf-enable=true "
                //         "! appsink name=streamSink sync=false",
                //         mCaptureDevice, mImageWidth, mImageHeight, mImageFramerate, mBitrate);

                launch = std::format(
                    "v4l2src device={} "
                    "! image/jpeg,width={},height={},framerate={}/1 "
                    "! nvv4l2decoder mjpeg=1 "
                    "! nvvidconv "
                    "! video/x-raw(memory:NVMM),format=NV12 "
                    "! nvv4l2h265enc name=encoder bitrate={} iframeinterval=300 vbv-size=33333 insert-sps-pps=true control-rate=constant_bitrate profile=Main num-B-Frames=0 ratecontrol-enable=true preset-level=UltraFastPreset EnableTwopassCBR=false maxperf-enable=true "
                    "! appsink name=streamSink sync=false",
                    mCaptureDevice, mImageWidth, mImageHeight, mImageFramerate, mBitrate);
            } else {
                launch = std::format(
                        "v4l2src device={} "
                        
                        // "! video/x-raw(memory:NVMM),format=YUY2,width={},height={},framerate={}/1 "
                        "! image/jpeg,width={},height={},framerate={}/1 "
                        "! nvv4l2decoder mjpeg=1 "

                        "! nvvidconv "
                        "! video/x-raw(memory:NVMM),format=NV12 "
                        "! nvv4l2h265enc name=encoder bitrate={} iframeinterval=300 vbv-size=33333 insert-sps-pps=true control-rate=constant_bitrate profile=Main num-B-Frames=0 ratecontrol-enable=true preset-level=UltraFastPreset EnableTwopassCBR=false maxperf-enable=true "
                        "! appsink name=streamSink sync=false",
                        mCaptureDevice, mImageWidth, mImageHeight, mImageFramerate, mBitrate);
            }
        }

        ROS_INFO_STREAM(std::format("GStreamer launch: {}", launch));
        mPipeline = gstCheck(gst_parse_launch(launch.c_str(), nullptr));

        if (mCaptureDevice.empty()) mImageSource = gstCheck(gst_bin_get_by_name(GST_BIN(mPipeline), "imageSource"));
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
            if (msg->width != mImageWidth || msg->height != mImageHeight) throw std::runtime_error{"Unsupported resolution"};

            // "step" is the number of bytes (NOT pixels) in an image row
            std::size_t size = msg->step * msg->height;
            GstBuffer* buffer = gstCheck(gst_buffer_new_allocate(nullptr, size, nullptr));
            GstMapInfo info;
            gst_buffer_map(buffer, &info, GST_MAP_WRITE);
            std::memcpy(info.data, msg->data.data(), size);
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

            mCaptureDevice = mPnh.param<std::string>("device", "");
            mImageTopic = mPnh.param<std::string>("image_topic", "image");
            mImageWidth = mPnh.param<int>("width", 640);
            mImageHeight = mPnh.param<int>("height", 480);
            mImageFramerate = mPnh.param<int>("framerate", 30);
            auto address = mPnh.param<std::string>("address", "0.0.0.0");
            auto port = mPnh.param<int>("port", 8081);
            mBitrate = mPnh.param<int>("bitrate", 2000000);

            gst_init(nullptr, nullptr);

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

            if (mCaptureDevice.empty()) {
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