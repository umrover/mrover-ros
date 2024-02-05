#include "software_h265_encoder.hpp"

#include <ros/init.h>

#include <sensor_msgs/Image.h>

#include "../../../deps/libde265/libde265/en265.h"

en265_encoder_context* encoder = nullptr;

auto imageCallback(const sensor_msgs::ImageConstPtr& msg) -> void {
    de265_image* image = en265_allocate_image(encoder, msg->width, msg->height, de265_chroma_420);
    if (image == nullptr) {
        ROS_ERROR("Failed to allocate image");
        ros::requestShutdown();
    }
    std::memcpy(image->data, msg->data.data(), msg->data.size());

    en265_push_image(encoder, image);
    en265_encode();

    en265_packet* packet = nullptr;
    while ((packet = en265_get_packet(encoder, 100)) != nullptr) {

        en265_free_packet(encoder, packet);
    }
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "software_h265_encoder");

    encoder = en265_new_encoder();
    if (encoder == nullptr) {
        ROS_ERROR("Failed to create encoder context");
        ros::requestShutdown();
    }

    en265_start_encoder(encoder);

    ros::spin();
    return EXIT_SUCCESS;
}