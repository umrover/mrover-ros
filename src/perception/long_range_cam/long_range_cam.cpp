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
            mCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>("camera/right/camera_info", 1);
            image_transport::ImageTransport it{mNh};
            mRightImgPub = it.advertise("long_range_cam/image", 1);

            int imageWidth{};
            int imageHeight{};
            // TODO: edit these so they correspond to the long range cam's resolution
            mPnh.param("image_width", imageWidth, 1280);
            mPnh.param("image_height", imageHeight, 720);

            if (imageWidth < 0 || imageHeight < 0) {
                throw std::invalid_argument("Invalid image dimensions");
            }
            if (mGrabTargetFps < 0) {
                throw std::invalid_argument("Invalid grab target framerate");
            }

            mImageResolution = sl::Resolution(imageWidth, imageHeight);
            mPointResolution = sl::Resolution(imageWidth, imageHeight);

            NODELET_INFO("Resolution: %s image: %zux%zu points: %zux%zu",
                         grabResolutionString.c_str(), mImageResolution.width, mImageResolution.height);

            sl::InitParameters initParameters;
            if (mSvoPath) {
                initParameters.input.setFromSVOFile(mSvoPath);
            } else {
                initParameters.input.setFromCameraID(-1, sl::BUS_TYPE::USB);
            }
            initParameters.depth_stabilization = mUseDepthStabilization;
            initParameters.camera_resolution = stringToZedEnum<sl::RESOLUTION>(grabResolutionString);
            initParameters.depth_mode = stringToZedEnum<sl::DEPTH_MODE>(depthModeString);
            initParameters.coordinate_units = sl::UNIT::METER;
            initParameters.sdk_verbose = true; // Log useful information
            initParameters.camera_fps = mGrabTargetFps;
            initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // Match ROS
            initParameters.depth_maximum_distance = mDepthMaximumDistance;

            if (mZed.open(initParameters) != sl::ERROR_CODE::SUCCESS) {
                throw std::runtime_error("ZED failed to open");
            }

            cudaDeviceProp prop{};
            cudaGetDeviceProperties(&prop, 0);
            ROS_INFO("MP count: %d, Max threads/MP: %d, Max blocks/MP: %d, max threads/block: %d",
                     prop.multiProcessorCount, prop.maxThreadsPerMultiProcessor, prop.maxBlocksPerMultiProcessor, prop.maxThreadsPerBlock);

            mGrabThread = std::thread(&ZedNodelet::grabUpdate, this);

        } catch (std::exception const& e) {
            NODELET_FATAL("Exception while starting: %s", e.what());
            ros::shutdown();
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
    void ZedNodelet::grabUpdate() {
        try {
            NODELET_INFO("Starting grab thread");
            while (ros::ok()) {
                mGrabThreadProfiler.beginLoop();

                sl::RuntimeParameters runtimeParameters;
                runtimeParameters.confidence_threshold = mDepthConfidence;
                runtimeParameters.texture_confidence_threshold = mTextureConfidence;

                if (mZed.grab(runtimeParameters) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to grab");
                mGrabThreadProfiler.measureEvent("Grab");

                // Retrieval has to happen on the same thread as grab so that the image and point cloud are synced
                if (mRightImgPub.getNumSubscribers())
                    if (mZed.retrieveImage(mGrabMeasures.rightImage, sl::VIEW::RIGHT, sl::MEM::GPU, mImageResolution) != sl::ERROR_CODE::SUCCESS)
                        throw std::runtime_error("ZED failed to retrieve right image");
                // Only left set is used for processing
                if (mZed.retrieveImage(mGrabMeasures.leftImage, sl::VIEW::LEFT, sl::MEM::GPU, mImageResolution) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to retrieve left image");
                if (mZed.retrieveMeasure(mGrabMeasures.leftPoints, sl::MEASURE::XYZ, sl::MEM::GPU, mPointResolution) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to retrieve point cloud");

                assert(mGrabMeasures.leftImage.timestamp == mGrabMeasures.leftPoints.timestamp);


                mGrabMeasures.time = mSvoPath ? ros::Time::now() : slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::IMAGE));
                mGrabThreadProfiler.measureEvent("Retrieve");

                // If the processing thread is busy skip
                // We want this thread to run as fast as possible for grab and positional tracking
                if (mSwapMutex.try_lock()) {
                    std::swap(mGrabMeasures, mPcMeasures);
                    mIsSwapReady = true;
                    mSwapMutex.unlock();
                    mSwapCv.notify_one();
                }
                mGrabThreadProfiler.measureEvent("Try swap");

                mGrabUpdateTick++;
            }

            mZed.close();
            NODELET_INFO("Grab thread finished");

        } catch (std::exception const& e) {
            NODELET_FATAL("Exception while running grab thread: %s", e.what());
            mZed.close();
            ros::shutdown();
            std::exit(EXIT_FAILURE);
        }
    }

    LongRangeCamNodelet::~LongRangeCamNodelet() {
        NODELET_INFO("Long range cam node shutting down");
        mPointCloudThread.join();
        mGrabThread.join();
    }

    ZedNodelet::Measures::Measures(ZedNodelet::Measures&& other) noexcept {
        *this = std::move(other);
    }

    ZedNodelet::Measures& ZedNodelet::Measures::operator=(ZedNodelet::Measures&& other) noexcept {
        sl::Mat::swap(other.leftImage, leftImage);
        sl::Mat::swap(other.rightImage, rightImage);
        sl::Mat::swap(other.leftPoints, leftPoints);
        std::swap(time, other.time);
        return *this;
    }
} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_wrapper");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet.load(ros::this_node::getName(), "mrover/ZedNodelet", ros::names::getRemappings(), {});

    ros::spin();

    return EXIT_SUCCESS;
}

#ifdef MROVER_IS_NODELET
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::ZedNodelet, nodelet::Nodelet)
#endif
