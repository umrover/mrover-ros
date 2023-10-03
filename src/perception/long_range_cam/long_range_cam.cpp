#include "long_range_cam.hpp"

namespace mrover {

    using namespace std::chrono_literals;

    /**
     * @brief Load config, open the camera, and start our threads
     */
    void LongRangeCamNodelet::onInit() {
        try {
            // See: http://wiki.ros.org/roscpp/Overview/NodeHandles for public vs. private node handle
            // MT means multithreaded
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();
            mCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);
            image_transport::ImageTransport it{mNh};
            mCamImgPub = it.advertise("long_range_cam/image", 1);
            
            // TODO: Make these camera open values ROS params/not hardcoded
            int deviceID = 0;
            int apiID = cv::CAP_ANY;
            mCapture.open(deviceID, apiID);
            if (!mCapture.isOpened()) {
                throw std::runtime_error("Long range cam failed to open");
            }
            
            mReadThread = std::thread(&LongRangeCamNodelet::readUpdate, this);
            mPubThread = std::thread(&LongRangeCamNodelet::imagePubUpdate, this);

        } catch (std::exception const& e) {
            NODELET_FATAL("Exception while starting: %s", e.what());
            ros::shutdown();
        }
    }

    /**
     * @brief Publishes images from long range cam.
     *
     * Takes in the image from the cam and publishes it.
     */
    void LongRangeCamNodelet::imagePubUpdate() {
        try {
            NODELET_INFO("Starting image pub thead");

            while (ros::ok()) {
                mPubThreadProfiler.beginLoop();

                // TODO: probably bad that this allocation, best case optimized by tcache
                // Needed because publish directly shares the pointer to other nodelets running in this process
                // TODO: What does this do?
                // Swap critical section
                {
                    std::unique_lock lock{mSwapMutex};
                    // Waiting on the condition variable will drop the lock and reacquire it when the condition is met
                    mSwapCv.wait(lock, [this] { return mIsSwapReady.load(); });
                    mIsSwapReady = false;
                    mPubThreadProfiler.measureEvent("Wait");
                    if (mLeftImgPub.getNumSubscribers()) {
                        auto imgMsg = boost::make_shared<sensor_msgs::Image>();
                        fillImageMessage(mPubMeasures.mImage, imgMsg);
                        imgMsg->header.frame_id = "long_range_cam_frame";
                        imgMsg->header.stamp = mPubMeasures.time;
                        imgMsg->header.seq = mImagePubUpdateTick;
                        mImgPub.publish(ImgMsg);

                        auto camInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();
                        fillCameraInfoMfessages(calibration, mImageResolution, leftCamInfoMsg, rightCamInfoMsg);
                        rightCamInfoMsg->header.frame_id = "long_range_cam_frame";
                        rightCamInfoMsg->header.stamp = mPcMeasures.time;
                        rightCamInfoMsg->header.seq = mImagePubUpdateTick;
                        mLeftCamInfoPub.publish(camInfoMsg);
                    }
                    
                    mPubThreadProfiler.measureEvent("Publish Message");
                }

                mImagePubUpdateTick++;
            }
            NODELET_INFO("Tag thread finished");

            }
        }
    }

    /**
     * @brief Grabs measures from the ZED.
     *
     * This update loop needs to happen as fast as possible.
     * grab() on the ZED updates positional tracking (visual odometry) which works best at high update rates.
     * As such we retrieve the image and point cloud on the GPU to send to the other thread for processing.
     * This only happens if the other thread is ready to avoid blocking, hence the try lock.
     */
    void LongRangeCam::readUpdate() {
        try {
            NODELET_INFO("Starting grab thread");
            while (ros::ok()) {
                mGrabThreadProfiler.beginLoop();
                // TODO: Combining grab and retrieve together, is that ok?
                if (!mCapture.read(mGrabMeasures, mImage))
                    throw std::runtime_error("Long range cam failed to get image");
                mGrabThreadProfiler.measureEvent("Read");

                mGrabMeasures.time = ros::Time::now();

                // If the processing thread is busy skip
                // We want this thread to run as fast as possible for grab and positional tracking
                if (mSwapMutex.try_lock()) {
                    std::swap(mGrabMeasures, mPubMeasures);
                    mIsSwapReady = true;
                    mSwapMutex.unlock();
                    mSwapCv.notify_one();
                }
                mGrabThreadProfiler.measureEvent("Try swap");

                mGrabUpdateTick++;
            }
            NODELET_INFO("Grab thread finished");

        } catch (std::exception const& e) {
            NODELET_FATAL("Exception while running grab thread: %s", e.what());
            ros::shutdown();
            std::exit(EXIT_FAILURE);
        }
    }

    LongRangeCamNodelet::~LongRangeCamNodelet() {
        NODELET_INFO("Long range cam node shutting down");
        mReadThread.join();
    }

    ZedNodelet::Measures::Measures(LongRangeCamNodelet::Measures&& other) noexcept {
        *this = std::move(other);
    }

    LongRangeCamNodelet::Measures& LongRangeCamNodelet::Measures::operator=(LongRangeCamNodelet::Measures&& other) noexcept {
        cv::OutpuArray::swap(other.mImage, mImage);
        std::swap(time, other.time);
        return *this;
    }
} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "long_range_cam_nodelet");

    // Start the Long Range Cam Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/LongRangeCamNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::LongRangeCamNodelet, nodelet::Nodelet)
#endif
