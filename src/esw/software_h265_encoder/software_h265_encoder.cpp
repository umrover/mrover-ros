#include "software_h265_encoder.hpp"

#include <ros/init.h>
#include <ros/node_handle.h>

#include <sensor_msgs/Image.h>

#include <libde265/en265.h>
#include <libde265/image.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <streaming.hpp>
#include <opencv2/highgui.hpp>
#include <ros/subscriber.h>

std::optional<StreamServer> streamServer;

en265_encoder_context* encoder = nullptr;

auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void {
    try {
        cv::Mat bgraImage{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<std::uint8_t*>(msg->data.data()), msg->step};
        cv::Mat yuv420Image;
        cvtColor(bgraImage, yuv420Image, cv::COLOR_BGRA2YUV_I420);

        de265_image* frame = en265_allocate_image(encoder, static_cast<int>(msg->width), static_cast<int>(msg->height), de265_chroma_420, 0, nullptr);
        if (frame == nullptr) {
            throw std::runtime_error{"Failed to allocate image"};
        }
        std::uint8_t* y = frame->get_image_plane(0);
        std::uint8_t* u = frame->get_image_plane(1);
        std::uint8_t* v = frame->get_image_plane(2);
        // int yStride = frame->get_image_stride(0);
        // int uStride = frame->get_image_stride(1);
        // int vStride = frame->get_image_stride(2);
        std::size_t area = bgraImage.total();
        std::memcpy(y, yuv420Image.data, area);
        std::memcpy(u, yuv420Image.data + area, area / 4);
        std::memcpy(v, yuv420Image.data + area * 5 / 4, area / 4);

        if (en265_push_image(encoder, frame) != DE265_OK) {
            throw std::runtime_error{"Failed to push image"};
        }
        if (en265_encode(encoder) != DE265_OK) {
            throw std::runtime_error{"Failed to encode"};
        }

        en265_packet* packet;
        while ((packet = en265_get_packet(encoder, 0)) != nullptr) {
            std::span span{std::bit_cast<std::byte*>(packet->data), static_cast<std::size_t>(packet->length)};
            ROS_INFO_STREAM(span.size());
            streamServer->feed(span);
            en265_free_packet(encoder, packet);
        }

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(std::format("Unhandled exception: {}", e.what()));
        ros::requestShutdown();
    }
}

auto main(int argc, char** argv) -> int {
    try {
        ros::init(argc, argv, "software_h265_encoder");
        ros::NodeHandle nh;

        streamServer.emplace("0.0.0.0", 8080);

        encoder = en265_new_encoder();
        if (encoder == nullptr) {
            throw std::runtime_error{"Failed to create encoder context"};
        }
        if (en265_start_encoder(encoder, 0) != DE265_OK) {
            throw std::runtime_error{"Failed to start encoder"};
        }

        ros::Subscriber imageSubscriber = nh.subscribe("/camera/left/image", 1, imageCallback);

        ros::spin();
        return EXIT_SUCCESS;

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(std::format("Unhandled exception: {}", e.what()));
        return EXIT_FAILURE;
    }
}