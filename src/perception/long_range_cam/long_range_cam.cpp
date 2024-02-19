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

    auto LongRangeCamNodelet::grabUpdate() -> void {
        try {
            NODELET_INFO("Starting USB grab thread...");

            // See: http://wiki.ros.org/roscpp/Overview/NodeHandles for public vs. private node handle
            // MT means multithreaded
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();
            mCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>("long_range_cam/camera_info", 1);
            mImgPub = mNh.advertise<sensor_msgs::Image>("long_range_image", 1);
            auto width = mPnh.param<int>("width", 1920);
            auto height = mPnh.param<int>("height", 1080);
            auto framerate = mPnh.param<int>("framerate", 5);
            auto device = mPnh.param<std::string>("device", "/dev/arducam");
            cv::VideoCapture capture{std::format("v4l2src device={} ! videoconvert ! video/x-raw,width={},height={},format=I420,framerate={}/1 ! appsink", device, width, height, framerate), cv::CAP_GSTREAMER};
            if (!capture.isOpened()) throw std::runtime_error{"Long range cam failed to open"};

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
        } catch (std::exception const& e) {
            NODELET_FATAL_STREAM(std::format("USB camera exception: {}", e.what()));
            ros::requestShutdown();
        }
    }

    LongRangeCamNodelet::~LongRangeCamNodelet() {
        NODELET_INFO("Long range cam node shutting down");
    }

} // namespace mrover
