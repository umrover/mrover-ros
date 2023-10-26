#include "long_range_cam.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <stdexcept>

namespace mrover {

    using namespace std::chrono_literals;

    // gst-launch-1.0 -e -v v4l2src device=/dev/video4 ! videoconvert ! videoscale ! videorate ! "video/x-raw,format=I420,width=640,height=480,framerate=30/1" ! x264enc key-int-max=15 ! rtph264pay ! udpsink host=localhost port=5000
    // gst-launch-1.0 -v udpsrc uri=udp://localhost:5000 ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! autovideosink
    /**
     * @brief Load config, open the camera, and start our threads
     */
    void LongRangeCamNodelet::onInit() {
        try {
            // See: http://wiki.ros.org/roscpp/Overview/NodeHandles for public vs. private node handle
            // MT means multithreaded
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();
            mCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>("long_range_cam/camera_info", 1);
            image_transport::ImageTransport it{mNh};
            mImgPub = it.advertise("long_range_cam/image", 1);

            // TODO: Fix hardcoded components of this string
            mCapture = cv::VideoCapture("v4l2src device=/dev/video4 ! videoconvert ! videoscale ! videorate ! 'video / x - raw, format = I420, width = 640, height = 480, framerate = 30 / 1' ! x264enc key-int-max=15 ! rtph264pay ! udpsink");
            cv::VideoWriter out = cv::VideoWriter("udpsrc ! application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! autovideosink host=localhost port =5000", cv::CAP_GSTREAMER, 0, 30, cv::Size(640, 480), true);
            if (!mCapture.isOpened()) {
                throw std::runtime_error("Long range cam failed to open");
            }
            ROS_ERROR("NODELET LOADED");
            cv::Mat frame;
            while (true) {
                mCapture.read(frame);
                if (frame.empty()) {
                    break;
                }
                out.write(frame);
                cv::imshow("Sender", frame);
                if (cv::waitKey(1) == 's') {
                    break;
                }
            }
        } catch (std::exception const& e) {
            NODELET_FATAL("Exception while starting: %s", e.what());
            ros::shutdown();
        }
    }

    LongRangeCamNodelet::~LongRangeCamNodelet() {
        NODELET_INFO("Long range cam node shutting down");
    }

} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "long_range_cam");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/LongRangeCamNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::LongRangeCamNodelet, nodelet::Nodelet)
