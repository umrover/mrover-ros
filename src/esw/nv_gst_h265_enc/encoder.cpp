#include "nv_gst_h265_enc.hpp"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <streaming.hpp>

#include <thread>

std::optional<StreamServer> streamServer;

GstElement *imageSource{}, *streamSink{}, *pipeline{};
GMainLoop* loop{};

std::thread gstMainLoopThread;
std::thread gstStreamSinkThread;

template<typename T>
auto gstCheck(T* t) -> T* {
    if (!t) throw std::runtime_error{"Failed to create"};
    return t;
}

auto gstCheck(gboolean b) -> void {
    if (!b) throw std::runtime_error{"Failed to create"};
}

auto pullStreamSamplesLoop() -> void {
    // Block until we receive a new H265 chunk from the encoder
    // This is okay since we are on a separate thread
    while (GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(streamSink))) {
        // Map the buffer so we can read it, passing it into the stream server which sends it over the network to any clients
        GstBuffer* buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        gst_buffer_map(buffer, &map, GST_MAP_READ);
        streamServer->feed({reinterpret_cast<std::byte*>(map.data), map.size});

        gst_buffer_unmap(buffer, &map);
        gst_sample_unref(sample);
    }
}

auto busMessageCallback(GstBus*, GstMessage* message, void*) -> gboolean {
    switch (GST_MESSAGE_TYPE(message)) {
        case GST_MESSAGE_ERROR: {
            GError* error;
            gchar* debug;
            gst_message_parse_error(message, &error, &debug);
            ROS_INFO_STREAM(std::format("Error: {} {}", error->message, debug));
            g_error_free(error);
            g_free(debug);
            g_main_loop_quit(loop);
            break;
        }
        case GST_MESSAGE_EOS:
            g_main_loop_quit(loop);
            break;
        default:
            break;
    }
    return TRUE;
}

auto initPipeline(std::uint32_t width, std::uint32_t height) -> void {
    ROS_INFO("Initializing and starting GStreamer pipeline...");

    loop = gstCheck(g_main_loop_new(nullptr, FALSE));
    std::string launch = std::format("appsrc name=imageSource ! video/x-raw,format=BGRA,width={},height={},framerate=30/1 ! videoconvert ! nvh265enc ! appsink name=streamSink", width, height);
    pipeline = gstCheck(gst_parse_launch(launch.c_str(), nullptr));

    imageSource = gstCheck(gst_bin_get_by_name(GST_BIN(pipeline), "imageSource"));
    streamSink = gstCheck(gst_bin_get_by_name(GST_BIN(pipeline), "streamSink"));

    GstBus* bus = gst_element_get_bus(pipeline);
    gst_bus_add_watch(bus, busMessageCallback, nullptr);

    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        throw std::runtime_error{"Failed to start"};

    ROS_INFO("Initialized and started GStreamer pipeline");

    gstMainLoopThread = std::thread{[] {
        ROS_INFO("Started GStreamer main loop");
        g_main_loop_run(loop);
        ROS_INFO("Stopped GStreamer main loop");
    }};

    gstStreamSinkThread = std::thread{[] {
        pullStreamSamplesLoop();
    }};
}

auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void {
    try {
        if (msg->encoding != sensor_msgs::image_encodings::BGRA8) throw std::runtime_error{"Unsupported encoding"};

        if (!pipeline) initPipeline(msg->width, msg->height);

        std::size_t size = msg->step * msg->height * 4;
        GstBuffer* buffer = gst_buffer_new_wrapped_full(
                GST_MEMORY_FLAG_READONLY,
                const_cast<std::uint8_t*>(msg->data.data()),
                size,
                0,
                size,
                nullptr,
                nullptr);
        GST_BUFFER_TIMESTAMP(buffer) = msg->header.stamp.sec * GST_SECOND + msg->header.stamp.nsec;
        GST_BUFFER_DURATION(buffer) = 1 * GST_SECOND / 30;

        if (gst_app_src_push_buffer(GST_APP_SRC(imageSource), buffer) != GST_FLOW_OK)
            throw std::runtime_error{"Failed to push buffer"};

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(std::format("Exception encoding frame: {}", e.what()));
        ros::requestShutdown();
    }
}

auto main(int argc, char** argv) -> int {
    try {
        ros::init(argc, argv, "nv_gst_h265_enc");
        ros::NodeHandle nh, pnh{"~"};

        auto imageTopic = pnh.param<std::string>("image_topic", "image");
        auto address = pnh.param<std::string>("address", "0.0.0.0");
        auto port = pnh.param<int>("port", 8080);

        gst_init(nullptr, nullptr);

        streamServer.emplace(address, port);

        ros::Subscriber imageSubscriber = nh.subscribe(imageTopic, 1, imageCallback);

        ros::spin();

        g_main_loop_quit(loop);

        gstMainLoopThread.join();
        gstStreamSinkThread.join();

        return EXIT_SUCCESS;

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(std::format("Exception initializing: {}", e.what()));
        return EXIT_FAILURE;
    }
}
