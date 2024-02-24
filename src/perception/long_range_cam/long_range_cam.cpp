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

    template<typename T>
    auto check(T* t) -> T* {
        if (!t) throw std::runtime_error{"Failed to create"};
        return t;
    }

    auto LongRangeCamNodelet::grabUpdate() -> void {
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
            auto doStream = mPnh.param<bool>("stream", true);
            auto bitrate = mPnh.param<int>("bitrate", 20000000);

            mImgPub = mNh.advertise<sensor_msgs::Image>(imageTopicName, 1);
            mCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>(cameraInfoTopicName, 1);

            std::string videoString = std::format("video/x-raw,format=I420,width={},height={},framerate={}/1", width, height, framerate);
            std::string gstString;
            if (doStream) {
                 gst_init(nullptr, nullptr);

                 GstElement* source = check(gst_element_factory_make("v4l2src", "source"));
                 GstElement* tee = check(gst_element_factory_make("tee", "tee"));
                 GstElement* videoConvert = check(gst_element_factory_make("videoconvert", "videoConvert"));
                 GstElement* videoConvert2 = check(gst_element_factory_make("videoconvert", "videoConvert2"));
                 GstElement* nvv4l2h265enc = check(gst_element_factory_make("nvv4l2h265enc", "nvv4l2h265enc"));
                 GstElement* appsink = check(gst_element_factory_make("appsink", "appsink"));
                 GstElement* appsink2 = check(gst_element_factory_make("appsink", "appsink2"));
                 g_object_set(G_OBJECT(source), "device", device.c_str(), nullptr);
                 g_object_set(G_OBJECT(nvv4l2h265enc), "bitrate", bitrate, nullptr);
                 g_object_set(G_OBJECT(appsink), "emit-signals", true, nullptr);

                 GstElement* pipeline = check(gst_pipeline_new("pipeline"));
                 gst_bin_add_many(GST_BIN(pipeline), source, tee, videoConvert, appsink, videoConvert2, nvv4l2h265enc, appsink2, nullptr);
            } else {
                gstString = std::format("v4l2src device={} ! videoconvert ! {} ! appsink", device, videoString);
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
                        cv::cvtColor(frame, bgra, cv::COLOR_YUV2BGRA_I420);
                        fillImageMessage(bgra, imageMessage);
                        imageMessage->header.frame_id = "long_range_cam_frame";
                        imageMessage->header.stamp = ros::Time::now();
                        mImgPub.publish(imageMessage);
                    }
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