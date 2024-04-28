#include "usb_camera.hpp"

namespace mrover {

    auto UsbCameraNodelet::onInit() -> void {
        mGrabThread = std::jthread(&UsbCameraNodelet::grabUpdate, this);
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

    auto UsbCameraNodelet::grabUpdate() -> void {
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

            mImgPub = mNh.advertise<sensor_msgs::Image>(imageTopicName, 1);
            mCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>(cameraInfoTopicName, 1);

            std::string captureFormat = std::format("video/x-raw,format=YUY2,width={},height={},framerate={}/1", width, height, framerate);
            std::string gstString = std::format("v4l2src device={} ! {} ! appsink", device, captureFormat);
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
                    cvtColor(frame, bgra, cv::COLOR_YUV2BGRA_YUY2);
                    fillImageMessage(bgra, imageMessage);
                    imageMessage->header.frame_id = "long_range_cam_frame";
                    imageMessage->header.stamp = ros::Time::now();
                    mImgPub.publish(imageMessage);
                }
            }

        } catch (std::exception const& e) {
            NODELET_FATAL_STREAM(std::format("USB camera exception: {}", e.what()));
            ros::requestShutdown();
        }
    }

    UsbCameraNodelet::~UsbCameraNodelet() {
        NODELET_INFO("Long range cam node shutting down");
    }

} // namespace mrover