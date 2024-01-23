#pragma once

#include "image_capture.hpp"
#include "pch.hpp"

namespace mrover {

    void ImageCaptureNodelet::onInit() {
        //Get the Handlers
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();

        m_fileLocation = "//home//john//Desktop//Rover//Images//image.jpg";

        //Create the publishers and subscribers for the detected image and the debug image
        mImgSub = mNh.subscribe("/camera/left/image", 1, &ImageCaptureNodelet::captureImageCallback, this);
    }

    void ImageCaptureNodelet::captureImageCallback(sensor_msgs::ImageConstPtr const& msg){
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        cv::Mat imageView{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC4, const_cast<uint8_t*>(msg->data.data())};

        cv::imwrite(m_fileLocation, imageView);

        ROS_INFO("width, height: %d, %d", static_cast<int>(msg->width), static_cast<int>(msg->height));
    }
} // namespace mrover
