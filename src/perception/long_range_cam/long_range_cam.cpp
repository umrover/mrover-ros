#include "long_range_cam.hpp"

namespace mrover {

    using namespace std::chrono_literals;

    /**
    * @brief Load config, open the camera, and start our threads
    */
    auto LongRangeCamNodelet::onInit() -> void {
        mGrabThread = std::jthread(&LongRangeCamNodelet::grabUpdate, this);
    }

    auto fillImageMessage(cv::Mat const& bgraImage, sensor_msgs::ImagePtr const& imageMessage) -> void {
        assert(!bgraImage.empty());
        assert(bgraImage.isContinuous());
        assert(bgraImage.type() == CV_8UC4);
        assert(imageMessage);

        imageMessage->height = bgraImage.rows;
        imageMessage->width = bgraImage.cols;
        imageMessage->encoding = sensor_msgs::image_encodings::BGRA8;
        imageMessage->step = bgraImage.step;
        imageMessage->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        size_t size = imageMessage->step * imageMessage->height;
        imageMessage->data.resize(size);
        std::memcpy(imageMessage->data.data(), bgraImage.data, size);
    }

    template<typename T>
    auto check(T* t) -> T* {
        if (!t) throw std::runtime_error{"Failed to create"};
        return t;
    }

    auto check(gboolean b) -> void {
        if (!b) throw std::runtime_error{"Failed to create"};
    }

    struct GstData {
        GstElement* pipeline{};
        GMainLoop* loop{};
    };

    static auto on_new_sample_from_sink(GstBus* bus, GstMessage* msg, GstData* data) -> void {
        switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR: {
                GError* err;
                gchar* debug_info;
                gst_message_parse_error(msg, &err, &debug_info);
                ROS_ERROR_STREAM(std::format("Error received from element {}: {}\n", GST_OBJECT_NAME(msg->src), err->message));
                ROS_ERROR_STREAM(std::format("Debugging information: {}\n", debug_info ? debug_info : "none"));
                g_error_free(err);
                g_free(debug_info);

                gst_element_set_state(data->pipeline, GST_STATE_READY);
                g_main_loop_quit(data->loop);
                break;
            }
            case GST_MESSAGE_EOS:
                gst_element_set_state(data->pipeline, GST_STATE_READY);
                g_main_loop_quit(data->loop);
                break;
            case GST_MESSAGE_CLOCK_LOST:
                gst_element_set_state(data->pipeline, GST_STATE_PAUSED);
                gst_element_set_state(data->pipeline, GST_STATE_PLAYING);
                break;
            default:
                break;
        }
    }

    static auto on_new_sample(GstElement* sink, gpointer* data) -> GstFlowReturn {
        GstSample* sample;
        g_signal_emit_by_name(sink, "pull-sample", &sample);
        if (sample) {
            if (GstBuffer* buffer = gst_sample_get_buffer(sample)) {
            }
            gst_sample_unref(sample);
            return GST_FLOW_OK;
        }
        return GST_FLOW_ERROR;
    }

    auto LongRangeCamNodelet::grabUpdate() -> void {
        try {
            NODELET_INFO("Starting USB grab thread...");

            // See: http://wiki.ros.org/roscpp/Overview/NodeHandles for public vs. private node handle
            // MT means multithreaded
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();

            auto width = mPnh.param<int>("width", 640);
            auto height = mPnh.param<int>("height", 480);
            auto framerate = mPnh.param<int>("framerate", 30);
            auto device = mPnh.param<std::string>("device", "/dev/video0");
            auto imageTopicName = mPnh.param<std::string>("image_topic", "/image");
            auto cameraInfoTopicName = mPnh.param<std::string>("camera_info_topic", "/camera_info");
            auto doStream = mPnh.param<bool>("stream", true);
            auto bitrate = mPnh.param<int>("bitrate", 2000000);

            mImgPub = mNh.advertise<sensor_msgs::Image>(imageTopicName, 1);
            mCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>(cameraInfoTopicName, 1);

            if (doStream) {
                gst_init(nullptr, nullptr);

                GstData data;

                GstElement* source = check(gst_element_factory_make("v4l2src", "source"));
                g_object_set(G_OBJECT(source), "device", device.c_str(), nullptr);
                GstElement* tee = check(gst_element_factory_make("tee", "tee"));
                GstElement* queue1 = check(gst_element_factory_make("queue", "queue1"));
                GstElement* queue2 = check(gst_element_factory_make("queue", "queue2"));
                GstElement* convert1 = check(gst_element_factory_make("videoconvert", "convert1"));
                GstElement* convert2 = check(gst_element_factory_make("videoconvert", "convert2"));
                // GstElement* encoder = check(gst_element_factory_make("nvv4l2h265enc", "encoder"));
                GstElement* encoder = check(gst_element_factory_make("nvh265enc", "encoder"));
                // GstElement* encoder = check(gst_element_factory_make("x265enc", "encoder"));
                GstElement* frameSink = check(gst_element_factory_make("appsink", "sink1"));
                GstElement* encoderSink = check(gst_element_factory_make("appsink", "sink2"));
                g_object_set(G_OBJECT(frameSink), "emit-signals", TRUE, "sync", FALSE, nullptr);
                g_object_set(G_OBJECT(encoderSink), "emit-signals", TRUE, "sync", FALSE, nullptr);
                // g_object_set(G_OBJECT(encoder), "bitrate", static_cast<guint>(bitrate / 1000), nullptr);

                data.pipeline = check(gst_pipeline_new("pipeline"));
                gst_bin_add_many(GST_BIN(data.pipeline), source, tee, queue1, convert1, frameSink, queue2, convert2, encoder, encoderSink, nullptr);

                check(gst_element_link(source, tee));

                gst_pad_link(
                        check(gst_element_get_request_pad(tee, "src_%u")),
                        check(gst_element_get_static_pad(queue1, "sink")));
                gst_pad_link(
                        check(gst_element_get_request_pad(tee, "src_%u")),
                        check(gst_element_get_static_pad(queue2, "sink")));

                GstCaps* caps = check(gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "I420", "width", G_TYPE_INT, width, "height", G_TYPE_INT, height, "framerate", GST_TYPE_FRACTION, framerate, 1, nullptr));

                check(gst_element_link(queue1, convert1));
                check(gst_element_link_filtered(convert1, frameSink, caps));

                check(gst_element_link(queue2, convert2));
                check(gst_element_link_filtered(convert2, encoder, caps));
                check(gst_element_link(encoder, encoderSink));

                GstBus* bus = check(gst_pipeline_get_bus(GST_PIPELINE(data.pipeline)));
                gst_bus_add_signal_watch(GST_BUS(bus));
                check(g_signal_connect(bus, "message", G_CALLBACK(on_new_sample_from_sink), &data));
                check(g_signal_connect(bus, "message", G_CALLBACK(on_new_sample_from_sink), &data));
                // check(g_signal_connect(frameSink, "new-sample", G_CALLBACK(on_new_sample), &data));
                check(g_signal_connect(encoderSink, "new-sample", G_CALLBACK(on_new_sample), &data));

                // gstString = std::format(
                //         "v4l2src device={0} ! tee name=t "
                //         "t. ! videoconvert ! {1} ! appsink"
                //         "t. ! videoconvert ! {1} ! nvv4l2h265enc bitrate={2} ! appsink",
                //         device, videoString, bitrate);
                // GstElement* pipeline = gst_parse_launch(gstString.c_str(), nullptr);
                //
                // GstBus* bus = gst_element_get_bus(pipeline);
                // GstMessage* msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
                // if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR) {
                //     GError* err;
                //     gchar* debug_info;
                //     gst_message_parse_error(msg, &err, &debug_info);
                //     g_printerr("Error received from element %s: %s\n", GST_OBJECT_NAME(msg->src), err->message);
                //     g_printerr("Debugging information: %s\n", debug_info ? debug_info : "none");
                //     g_clear_error(&err);
                //     g_free(debug_info);
                // }

                if (gst_element_set_state(data.pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
                    throw std::runtime_error{"Failed to start pipeline"};

                data.loop = g_main_loop_new(nullptr, FALSE);
                g_main_loop_run(data.loop);

                throw std::runtime_error{"Pipeline exited early"};

            } else {
                std::string videoString = std::format("video/x-raw,format=I420,width={},height={},framerate={}/1", width, height, framerate);
                std::string gstString = std::format("v4l2src device={} ! videoconvert ! {} ! appsink", device, videoString);
                NODELET_INFO_STREAM(std::format("GStreamer string: {}", gstString));
                cv::VideoCapture capture{gstString, cv::CAP_GSTREAMER};
                if (!capture.isOpened()) throw std::runtime_error{"USB camera failed to open"};

                NODELET_INFO_STREAM(std::format("USB camera opened: {}x{} @ {} fps", width, height, framerate));

                cv::Mat frame;
                while (ros::ok()) {
                    capture.read(frame);
                    if (frame.empty()) break;

                    if (mImgPub.getNumSubscribers()) {
                        auto imageMessage = boost::make_shared<sensor_msgs::Image>();
                        cv::Mat bgra;
                        cv::cvtColor(frame, bgra, cv::COLOR_YUV2BGRA_I420);
                        fillImageMessage(bgra, imageMessage);
                        imageMessage->header.frame_id = "long_range_cam_frame";
                        imageMessage->header.stamp = ros::Time::now();
                        mImgPub.publish(imageMessage);
                    }
                }
            }
        } catch (std::exception const& e) {
            NODELET_FATAL_STREAM(std::format("USB camera exception: {}", e.what()));
            ros::requestShutdown();
        }
    }

    LongRangeCamNodelet::~LongRangeCamNodelet() {
        NODELET_INFO("Long range cam node shutting down");
    }

} // namespace mrover