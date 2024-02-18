#include "hardware_h265_encoder.hpp"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <streaming.hpp>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

std::optional<StreamServer> streamServer;

std::optional<Encoder> encoder;

auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void {
    try {
        if (msg->encoding != sensor_msgs::image_encodings::BGR8) throw std::runtime_error{"Unsupported encoding"};

        cv::Size size{static_cast<int>(msg->width), static_cast<int>(msg->height)};

        if (!encoder) encoder.emplace(size);

        cv::Mat bgrFrame{size, CV_8UC3, const_cast<std::uint8_t*>(msg->data.data()), msg->step};
        cv::Mat bgraFrame;
        cv::cvtColor(bgrFrame, bgraFrame, cv::COLOR_BGR2BGRA);

        Encoder::BitstreamView view = encoder->feed(bgraFrame);
        std::span span{static_cast<std::byte*>(view.lockParams.bitstreamBufferPtr), view.lockParams.bitstreamSizeInBytes};
        streamServer->feed(span);

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(std::format("Exception encoding frame: {}", e.what()));
        ros::requestShutdown();
    }
}

// auto capture() -> void {
//     cv::VideoCapture cap{std::format("v4l2src ! videoconvert ! video/x-raw,width={},height={},format=I420,framerate=30/1 ! appsink", 640, 480), cv::CAP_GSTREAMER};
//     if (!cap.isOpened()) throw std::runtime_error{"Failed to open capture"};
//
//     while (cap.isOpened()) {
//         cv::Mat i420Frame;
//         if (!cap.read(i420Frame)) throw std::runtime_error{"Failed to read frame"};
//
//         cv::Mat bgraFrame;
//         cvtColor(i420Frame, bgraFrame, cv::COLOR_YUV2BGRA_I420);
//
//         Encoder::BitstreamView view = encoder->feed(bgraFrame);
//         std::span span{static_cast<std::byte*>(view.lockParams.bitstreamBufferPtr), view.lockParams.bitstreamSizeInBytes};
//         streamServer->feed(span);
//
//         ros::spinOnce();
//     }
// }

auto main(int argc, char** argv) -> int {
    try {
        ros::init(argc, argv, "software_h265_encoder");
        ros::NodeHandle nh, pnh{"~"};

        std::string imageTopic = pnh.param("image_topic", std::string{"/camera/left/image"});

        streamServer.emplace("0.0.0.0", 8080);

        ros::Subscriber imageSubscriber = nh.subscribe(imageTopic, 1, imageCallback);

        // capture();

        ros::spin();
        return EXIT_SUCCESS;

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(std::format("Exception initializing: {}", e.what()));
        return EXIT_FAILURE;
    }
}
