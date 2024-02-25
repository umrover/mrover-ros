#include "nv_gst_h265_enc.hpp"

namespace mrover {

    template<typename T>
    auto gstCheck(T* t) -> T* {
        if (!t) throw std::runtime_error{"Failed to create"};
        return t;
    }

    auto gstCheck(gboolean b) -> void {
        if (!b) throw std::runtime_error{"Failed to create"};
    }

    auto NvGstH265Enc::pullStreamSamplesLoop() -> void {
        // Block until we receive a new H265 chunk from the encoder
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
        auto encoder = static_cast<NvGstH265Enc*>(userData);
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

    auto NvGstH265Enc::initPipeline(std::uint32_t width, std::uint32_t height) -> void {
        ROS_INFO("Initializing and starting GStreamer pipeline...");

        mMainLoop = gstCheck(g_main_loop_new(nullptr, FALSE));
        std::string launch = std::format("appsrc name=imageSource ! video/x-raw,format=BGRA,width={},height={},framerate=30/1 ! videoconvert ! nvh265enc ! appsink name=streamSink", width, height);
        mPipeline = gstCheck(gst_parse_launch(launch.c_str(), nullptr));

        mImageSource = gstCheck(gst_bin_get_by_name(GST_BIN(mPipeline), "imageSource"));
        mStreamSink = gstCheck(gst_bin_get_by_name(GST_BIN(mPipeline), "streamSink"));

        GstBus* bus = gst_element_get_bus(mPipeline);
        gst_bus_add_watch(bus, busMessageCallback, this);
        gst_object_unref(bus);

        if (gst_element_set_state(mPipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
            throw std::runtime_error{"Failed to start"};

        ROS_INFO("Initialized and started GStreamer pipeline");

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
    }

    auto NvGstH265Enc::imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void {
        try {
            if (msg->encoding != sensor_msgs::image_encodings::BGRA8) throw std::runtime_error{"Unsupported encoding"};

            // TODO(quintin): Do this better
            mStreamServer->feed({});

            if (!mPipeline) initPipeline(msg->width, msg->height);

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

    auto NvGstH265Enc::onInit() -> void {
        try {
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();

            auto imageTopic = mNh.param<std::string>("image_topic", "image");
            auto address = mPnh.param<std::string>("address", "0.0.0.0");
            auto port = mPnh.param<int>("port", 8080);

            gst_init(nullptr, nullptr);

            mStreamServer.emplace(address, port);

            mImageSubscriber = mNh.subscribe(imageTopic, 1, &NvGstH265Enc::imageCallback, this);

        } catch (std::exception const& e) {
            ROS_ERROR_STREAM(std::format("Exception initializing NVIDIA GST H265 Encoder: {}", e.what()));
            ros::requestShutdown();
        }
    }

    NvGstH265Enc::~NvGstH265Enc() {
        gst_app_src_end_of_stream(GST_APP_SRC(mImageSource));

        mMainLoopThread.join();
        mStreamSinkThread.join();

        gst_element_set_state(mPipeline, GST_STATE_NULL);
        gst_object_unref(mPipeline);
        gst_object_unref(mImageSource);
        gst_object_unref(mStreamSink);
    }

} // namespace mrover
