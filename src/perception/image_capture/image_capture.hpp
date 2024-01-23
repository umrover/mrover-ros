#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include "pch.hpp"
namespace mrover{

    class ImageCaptureNodelet : public nodelet::Nodelet {
    private:
        //Image Container
        cv::Mat image;

        //Location to save images
        std::filesystem::path m_fileLocation;

        //ROS Handlers
        ros::NodeHandle mNh, mPnh;

        // Subscribers
        ros::Subscriber mImgSub;

        //Init method
        void onInit() override;

    public:
        ImageCaptureNodelet() = default;

        ~ImageCaptureNodelet() override = default;

        void captureImageCallback(sensor_msgs::ImageConstPtr const& msg);
    };

}//namespace mrover