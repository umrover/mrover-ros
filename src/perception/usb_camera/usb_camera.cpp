#include "usb_camera.hpp"

namespace mrover {

    template<typename T>
    auto gstCheck(T* t) -> T* {
        if (!t) throw std::runtime_error{"Failed to create"};
        return t;
    }

    auto UsbCameraNodelet::onInit() -> void {
        try {
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();

            mWidth = mPnh.param<int>("width", 640);
            mHeight = mPnh.param<int>("height", 480);
            auto framerate = mPnh.param<int>("framerate", 30);
            auto device = mPnh.param<std::string>("device", "/dev/video0");
            auto imageTopicName = mPnh.param<std::string>("image_topic", "/image");
            auto cameraInfoTopicName = mPnh.param<std::string>("camera_info_topic", "/camera_info");

            mImgPub = mNh.advertise<sensor_msgs::Image>(imageTopicName, 1);
            mCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>(cameraInfoTopicName, 1);

            gst_init(nullptr, nullptr);

            mMainLoop = gstCheck(g_main_loop_new(nullptr, FALSE));

            std::string captureFormat = std::format("video/x-raw,format=YUY2,width={},height={},framerate={}/1", mWidth, mHeight, framerate);
            std::string launch = std::format("v4l2src device={} ! {} ! appsink name=streamSink sync=false", device, captureFormat);
            NODELET_INFO_STREAM(std::format("GStreamer launch string: {}", launch));
            mPipeline = gstCheck(gst_parse_launch(launch.c_str(), nullptr));

            mStreamSink = gstCheck(gst_bin_get_by_name(GST_BIN(mPipeline), "streamSink"));

            if (gst_element_set_state(mPipeline, GST_STATE_PAUSED) == GST_STATE_CHANGE_FAILURE)
                throw std::runtime_error{"Failed initial pause on GStreamer pipeline"};

            mMainLoopThread = std::thread{[this] {
                ROS_INFO("Started GStreamer main loop");
                g_main_loop_run(mMainLoop);
                std::cout << "Stopped GStreamer main loop" << std::endl;
            }};

            mStreamSinkThread = std::thread{[this] {
                ROS_INFO("Started stream sink thread");
                pullSampleLoop();
                std::cout << "Stopped stream sink thread" << std::endl;
            }};

            if (gst_element_set_state(mPipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
                throw std::runtime_error{"Failed to play GStreamer pipeline"};

            NODELET_INFO_STREAM("Initialized and started GStreamer pipeline");

        } catch (std::exception const& e) {
            ROS_ERROR_STREAM(std::format("Exception initializing USB camera: {}", e.what()));
            ros::requestShutdown();
        }
    }

    auto UsbCameraNodelet::pullSampleLoop() const -> void {
        while (GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(mStreamSink))) {
            GstBuffer* buffer = gst_sample_get_buffer(sample);
            GstMapInfo map;
            gst_buffer_map(buffer, &map, GST_MAP_READ);

            sensor_msgs::Image image;
            image.header.stamp = ros::Time::now();
            image.encoding = sensor_msgs::image_encodings::BGRA8;
            image.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
            image.height = mHeight;
            image.width = mWidth;
            image.step = mWidth * 4;
            image.data.resize(image.step * mHeight);
            cv::cvtColor(cv::Mat{mHeight, mWidth, CV_8UC2, map.data}, cv::Mat{mHeight, mWidth, CV_8UC4, image.data.data()}, cv::COLOR_YUV2BGRA_YUY2);
            mImgPub.publish(image);

            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
        }
    }

    UsbCameraNodelet::~UsbCameraNodelet() {
        if (mMainLoop) {
            g_main_loop_quit(mMainLoop);
            mMainLoopThread.join();
            g_main_loop_unref(mMainLoop);
        }

        if (mPipeline) {
            gst_element_set_state(mPipeline, GST_STATE_NULL);
            mStreamSinkThread.join();
            gst_object_unref(mPipeline);
        }
    }

} // namespace mrover