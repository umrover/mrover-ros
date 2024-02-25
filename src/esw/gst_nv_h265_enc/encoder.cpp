#include "gst_nv_h265_enc.hpp"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <streaming.hpp>

#include <thread>

std::optional<StreamServer> streamServer;

GstElement *imageSource{}, *streamSink{}, *pipeline{};
GMainLoop* loop{};

std::thread gstLoopThread;

template<typename T>
auto gstCheck(T* t) -> T* {
    if (!t) throw std::runtime_error{"Failed to create"};
    return t;
}

auto gstCheck(gboolean b) -> void {
    if (!b) throw std::runtime_error{"Failed to create"};
}

auto initPipeline(std::uint32_t width, std::uint32_t height) -> void {
    ROS_INFO("Initializing GStreamer pipeline...");

    loop = gstCheck(g_main_loop_new(nullptr, FALSE));
    std::string launch = std::format("appsrc name=imageSource ! video/x-raw,format=BGRA,width={},height={},framerate=30/1 ! videoconvert ! nvh265enc ! appsink name=streamSink", width, height);
    pipeline = gstCheck(gst_parse_launch(launch.c_str(), nullptr));

    imageSource = gstCheck(gst_bin_get_by_name(GST_BIN(pipeline), "imageSource"));
    streamSink = gstCheck(gst_bin_get_by_name(GST_BIN(pipeline), "streamSink"));

    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
        throw std::runtime_error{"Failed to start"};

    gstLoopThread = std::thread{[] { g_main_loop_run(loop); }};

    ROS_INFO("GStreamer Pipeline and Main Loop Started");
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

        GstFlowReturn pushResult;
        g_signal_emit_by_name(imageSource, "push-buffer", buffer, &pushResult);

        gst_buffer_unref(buffer);

        if (pushResult != GST_FLOW_OK) throw std::runtime_error{"Failed to push buffer"};

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(std::format("Exception encoding frame: {}", e.what()));
        ros::requestShutdown();
    }
}

auto main(int argc, char** argv) -> int {
    try {
        ros::init(argc, argv, "gst_nv_h265_enc");
        ros::NodeHandle nh, pnh{"~"};

        std::string imageTopic = pnh.param("image_topic", std::string{"/image"});

        gst_init(nullptr, nullptr);

        streamServer.emplace("0.0.0.0", 8080);

        ros::Subscriber imageSubscriber = nh.subscribe(imageTopic, 1, imageCallback);

        ros::spin();

        g_signal_emit_by_name(imageSource, "end-of-stream", nullptr);

        g_main_loop_quit(loop);

        gstLoopThread.join();

        return EXIT_SUCCESS;

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(std::format("Exception initializing: {}", e.what()));
        return EXIT_FAILURE;
    }
}
