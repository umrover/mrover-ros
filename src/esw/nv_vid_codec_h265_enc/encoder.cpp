#include "nv_vid_codec_h265_enc.hpp"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <streaming.hpp>

#include <opencv2/imgproc.hpp>

std::optional<StreamServer> streamServer;

std::optional<NvVideoCodecEncoder> encoder;

auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void {
    try {
        if (msg->encoding != sensor_msgs::image_encodings::BGRA8) throw std::runtime_error{"Unsupported encoding"};

        cv::Size size{static_cast<int>(msg->width), static_cast<int>(msg->height)};

        if (!encoder) encoder.emplace(size);

        cv::Mat bgraFrame{size, CV_8UC4, const_cast<std::uint8_t*>(msg->data.data()), msg->step};

        bool feedSuccessful;
        {
            NvVideoCodecEncoder::BitstreamView view = encoder->feed(bgraFrame);
            std::span span{static_cast<std::byte*>(view.lockParams.bitstreamBufferPtr), view.lockParams.bitstreamSizeInBytes};
            feedSuccessful = streamServer->feed(span);
        }
        if (!feedSuccessful) encoder.reset();

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(std::format("Exception encoding frame: {}", e.what()));
        ros::requestShutdown();
    }
}

auto main(int argc, char** argv) -> int {
    try {
        ros::init(argc, argv, "nv_vid_codec_h265_enc");
        ros::NodeHandle nh, pnh{"~"};

        std::string imageTopic = pnh.param("image_topic", std::string{"/camera/left/image"});

        streamServer.emplace("0.0.0.0", 8080);

        ros::Subscriber imageSubscriber = nh.subscribe(imageTopic, 1, imageCallback);

        ros::spin();
        return EXIT_SUCCESS;

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(std::format("Exception initializing: {}", e.what()));
        return EXIT_FAILURE;
    }
}
