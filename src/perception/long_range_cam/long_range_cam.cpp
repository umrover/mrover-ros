#include "long_range_cam.hpp"
#include <opencv2/videoio.hpp>
#include <ros/init.h>
#include <ros/this_node.h>
#include <ros/time.h>
#include <sensor_msgs/image_encodings.h>
#include <thread>

namespace mrover {

    using namespace std::chrono_literals;


    // gst-launch-1.0 -e -v v4l2src device=/dev/video4 ! videoconvert ! videoscale ! videorate ! "video/x-raw,format=I420,width=640,height=480,framerate=30/1" ! x264enc key-int-max=15 ! rtph264pay ! udpsink host=localhost port=5000
    // gst-launch-1.0 -v udpsrc uri=udp://localhost:5000 ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! autovideosink
    /**
        * @brief Load config, open the camera, and start our threads
        */
    void LongRangeCamNodelet::onInit() {
        try {
            mGrabThread = std::jthread(&LongRangeCamNodelet::grabUpdate, this);
        } catch (std::exception const& e) {
            NODELET_FATAL("Exception while starting: %s", e.what());
            ros::shutdown();
        }
    }

    void fillImageMessage(cv::Mat& grey, sensor_msgs::ImagePtr const& msg) {
        // TODO: Uncomment this
        // assert(img.channels() == 4);
        assert(msg);

        msg->height = grey.rows;
        msg->width = grey.cols;
        msg->encoding = sensor_msgs::image_encodings::MONO8;
        msg->step = grey.step[0];
        msg->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        auto* imgPtr = grey.ptr<float>(0);
        size_t size = msg->step * msg->height;
        msg->data.resize(size);
        std::cout << "data size " << msg->data.size() << std::endl;
        std::cout << "size obj " << size << std::endl;
        memcpy(msg->data.data(), imgPtr, size);
    }

    void LongRangeCamNodelet::grabUpdate() {
        // See: http://wiki.ros.org/roscpp/Overview/NodeHandles for public vs. private node handle
        // MT means multithreaded
        mNh = getMTNodeHandle();
        mPnh = getMTPrivateNodeHandle();
        mCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>("long_range_cam/camera_info", 1);
        mImgPub = mNh.advertise<sensor_msgs::Image>("long_range_cam/image", 1);
        // While dtor not called
        while (ros::ok()) {
            cv::VideoCapture mCapture{std::format("v4l2src device=/dev/video4 ! videoconvert ! video/x-raw,width={},height={},format=I420,framerate={}/1 ! appsink", 1920, 1080, 5), cv::CAP_GSTREAMER};
            if (!mCapture.isOpened()) {
                throw std::runtime_error("Long range cam failed to open");
            }
            cv::Mat frame;
            while (true) {
                mCapture.read(frame);
                if (frame.empty()) {
                    break;
                }
                cv::imshow("Sender", frame);
                if (mImgPub.getNumSubscribers()) {
                    auto imgMsg = boost::make_shared<sensor_msgs::Image>();
                    cv::Mat grey;
                    cv::cvtColor(frame, grey, cv::COLOR_YUV2GRAY_I420);
                    fillImageMessage(grey, imgMsg);
                    imgMsg->header.frame_id = "long_range_cam_frame";
                    imgMsg->header.stamp = ros::Time::now();
                    imgMsg->header.seq = mGrabUpdateTick;
                    mImgPub.publish(imgMsg);
                }
                if (cv::waitKey(1) == 's') {
                    break;
                }
                ++mGrabUpdateTick;
            }
        }
    }

    LongRangeCamNodelet::~LongRangeCamNodelet() {
        NODELET_INFO("Long range cam node shutting down");
    }

} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "usb_camera");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    std::cout << ros::this_node::getName() << std::endl;
    nodelet.load(ros::this_node::getName(), "mrover/LongRangeCamNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::LongRangeCamNodelet, nodelet::Nodelet)
#endif
