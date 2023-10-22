#include "zed_wrapper.hpp"

namespace mrover {

    using namespace std::chrono_literals;

    /**
     * Allows us to store enums as strings in the config file.
     * This avoids problems when ZED updates their enum integer values.
     *
     * @tparam TEnum    ZED enum type
     * @param string    String to convert to enum
     * @return          Enum value
     */
    template<typename TEnum>
    [[nodiscard]] TEnum stringToZedEnum(std::string_view string) {
        using int_t = std::underlying_type_t<TEnum>;
        for (int_t i = 0; i < static_cast<int_t>(TEnum::LAST); ++i) {
            if (sl::String{string.data()} == sl::toString(static_cast<TEnum>(i))) {
                return static_cast<TEnum>(i);
            }
        }
        throw std::invalid_argument("Invalid enum string");
    }

    /**
     * @brief Load config, open the ZED, and start our threads
     */
    void ZedNodelet::onInit() {
        try {
            // See: http://wiki.ros.org/roscpp/Overview/NodeHandles for public vs. private node handle
            // MT means multithreaded
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();
            mPcPub = mNh.advertise<sensor_msgs::PointCloud2>("camera/left/points", 1);
            mImuPub = mNh.advertise<sensor_msgs::Imu>("imu", 1);
            mMagPub = mNh.advertise<sensor_msgs::MagneticField>("mag", 1);
            mLeftCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>("camera/left/camera_info", 1);
            mRightCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>("camera/right/camera_info", 1);
            mLeftImgPub = mNh.advertise<sensor_msgs::Image>("camera/left/image", 1);
            mRightImgPub = mNh.advertise<sensor_msgs::Image>("camera/right/image", 1);

            std::string grabResolutionString;
            mPnh.param("grab_resolution", grabResolutionString, std::string{sl::toString(sl::RESOLUTION::HD720)});
            std::string depthModeString{};
            mPnh.param("depth_mode", depthModeString, std::string{sl::toString(sl::DEPTH_MODE::PERFORMANCE)});
            mPnh.param("grab_target_fps", mGrabTargetFps, 50);
            int imageWidth{};
            int imageHeight{};
            mPnh.param("image_width", imageWidth, 1280);
            mPnh.param("image_height", imageHeight, 720);
            mPnh.param("depth_confidence", mDepthConfidence, 70);
            mPnh.param("texture_confidence", mTextureConfidence, 100);
            std::string svoFile{};
            mPnh.param("svo_file", svoFile, {});
            mSvoPath = svoFile.c_str();
            mPnh.param("use_builtin_visual_odom", mUseBuiltinPosTracking, false);
            mPnh.param("use_area_memory", mUseAreaMemory, true);
            mPnh.param("use_pose_smoothing", mUsePoseSmoothing, true);
            mPnh.param("use_loop_profiler", mUseLoopProfiler, true);
            mPnh.param("use_depth_stabilization", mUseDepthStabilization, false);
            mPnh.param("depth_maximum_distance", mDepthMaximumDistance, 12.0f);

            if (imageWidth < 0 || imageHeight < 0) {
                throw std::invalid_argument("Invalid image dimensions");
            }
            if (mGrabTargetFps < 0) {
                throw std::invalid_argument("Invalid grab target framerate");
            }

            mImageResolution = sl::Resolution(imageWidth, imageHeight);
            mPointResolution = sl::Resolution(imageWidth, imageHeight);

            NODELET_INFO("Resolution: %s image: %zux%zu points: %zux%zu",
                         grabResolutionString.c_str(), mImageResolution.width, mImageResolution.height, mPointResolution.width, mPointResolution.height);
            NODELET_INFO("Use builtin visual odometry: %s", mUseBuiltinPosTracking ? "true" : "false");

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
            mZedInfo = mZed.getCameraInformation();

            if (mUseBuiltinPosTracking) {
                sl::PositionalTrackingParameters positionalTrackingParameters;
                positionalTrackingParameters.enable_pose_smoothing = mUsePoseSmoothing;
                positionalTrackingParameters.enable_area_memory = mUseAreaMemory;
                mZed.enablePositionalTracking(positionalTrackingParameters);
            }

            cudaDeviceProp prop{};
            cudaGetDeviceProperties(&prop, 0);
            ROS_INFO("MP count: %d, Max threads/MP: %d, Max blocks/MP: %d, max threads/block: %d",
                     prop.multiProcessorCount, prop.maxThreadsPerMultiProcessor, prop.maxBlocksPerMultiProcessor, prop.maxThreadsPerBlock);

            mGrabThread = std::thread(&ZedNodelet::grabUpdate, this);
            mPointCloudThread = std::thread(&ZedNodelet::pointCloudUpdate, this);

        } catch (std::exception const& e) {
            NODELET_FATAL("Exception while starting: %s", e.what());
            ros::shutdown();
        }
    }

    /**
     * @brief Processes grabbed data from ZED.
     *
     * Takes in the GPU pointers to the image and the point cloud.
     * It fuses these into a point cloud 2 message which is published.
     */
    void ZedNodelet::pointCloudUpdate() {
        try {
            NODELET_INFO("Starting point cloud thread");

            while (ros::ok()) {
                mPcThreadProfiler.beginLoop();

                // TODO: probably bad that this allocation, best case optimized by tcache
                // Needed because publish directly shares the pointer to other nodelets running in this process
                auto pointCloudMsg = boost::make_shared<sensor_msgs::PointCloud2>();

                // Swap critical section
                {
                    std::unique_lock lock{mSwapMutex};
                    // Waiting on the condition variable will drop the lock and reacquire it when the condition is met
                    mSwapCv.wait(lock, [this] { return mIsSwapReady.load(); });
                    mIsSwapReady = false;
                    mPcThreadProfiler.measureEvent("Wait");

                    fillPointCloudMessageFromGpu(mPcMeasures.leftPoints, mPcMeasures.leftImage, mPointCloudGpu, pointCloudMsg);
                    pointCloudMsg->header.seq = mPointCloudUpdateTick;
                    pointCloudMsg->header.stamp = mPcMeasures.time;
                    pointCloudMsg->header.frame_id = "zed2i_left_camera_frame";
                    mPcThreadProfiler.measureEvent("Fill Message");

                    if (mLeftImgPub.getNumSubscribers()) {
                        auto leftImgMsg = boost::make_shared<sensor_msgs::Image>();
                        fillImageMessage(mPcMeasures.leftImage, leftImgMsg);
                        leftImgMsg->header.frame_id = "zed2i_left_camera_optical_frame";
                        leftImgMsg->header.stamp = mPcMeasures.time;
                        leftImgMsg->header.seq = mPointCloudUpdateTick;
                        mLeftImgPub.publish(leftImgMsg);
                    }
                    if (mRightImgPub.getNumSubscribers()) {
                        auto rightImgMsg = boost::make_shared<sensor_msgs::Image>();
                        fillImageMessage(mPcMeasures.rightImage, rightImgMsg);
                        rightImgMsg->header.frame_id = "zed2i_right_camera_optical_frame";
                        rightImgMsg->header.stamp = mPcMeasures.time;
                        rightImgMsg->header.seq = mPointCloudUpdateTick;
                        mRightImgPub.publish(rightImgMsg);
                    }
                    mPcThreadProfiler.measureEvent("Publish Message");
                }

                if (mPcPub.getNumSubscribers()) {
                    mPcPub.publish(pointCloudMsg);
                    mPcThreadProfiler.measureEvent("Point cloud publish");
                }

                if (mLeftCamInfoPub.getNumSubscribers() || mRightCamInfoPub.getNumSubscribers()) {
                    sl::CalibrationParameters calibration = mZedInfo.camera_configuration.calibration_parameters;
                    auto leftCamInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();
                    auto rightCamInfoMsg = boost::make_shared<sensor_msgs::CameraInfo>();
                    fillCameraInfoMessages(calibration, mImageResolution, leftCamInfoMsg, rightCamInfoMsg);
                    leftCamInfoMsg->header.frame_id = "zed2i_left_camera_optical_frame";
                    leftCamInfoMsg->header.stamp = mPcMeasures.time;
                    leftCamInfoMsg->header.seq = mPointCloudUpdateTick;
                    rightCamInfoMsg->header.frame_id = "zed2i_right_camera_optical_frame";
                    rightCamInfoMsg->header.stamp = mPcMeasures.time;
                    rightCamInfoMsg->header.seq = mPointCloudUpdateTick;
                    mLeftCamInfoPub.publish(leftCamInfoMsg);
                    mRightCamInfoPub.publish(rightCamInfoMsg);
                    mPcThreadProfiler.measureEvent("Image + camera info publish");
                }

                mPointCloudUpdateTick++;
            }
            NODELET_INFO("Tag thread finished");

        } catch (std::exception const& e) {
            NODELET_FATAL("Exception while running tag thread: %s", e.what());
            ros::shutdown();
            std::exit(EXIT_FAILURE);
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

                // Positional tracking module publishing
                if (mUseBuiltinPosTracking) {
                    sl::Pose pose;
                    sl::POSITIONAL_TRACKING_STATE status = mZed.getPosition(pose);
                    if (status == sl::POSITIONAL_TRACKING_STATE::OK) {
                        sl::Translation const& translation = pose.getTranslation();
                        sl::Orientation const& orientation = pose.getOrientation();
                        try {
                            SE3 leftCameraInOdom{{translation.x, translation.y, translation.z},
                                                 Eigen::Quaterniond{orientation.w, orientation.x, orientation.y, orientation.z}.normalized()};
                            SE3 leftCameraInBaseLink = SE3::fromTfTree(mTfBuffer, "base_link", "zed2i_left_camera_frame");
                            SE3 baseLinkInOdom = leftCameraInBaseLink * leftCameraInOdom;
                            SE3::pushToTfTree(mTfBroadcaster, "base_link", "odom", baseLinkInOdom);
                        } catch (tf2::TransformException& e) {
                            NODELET_WARN_STREAM("Failed to get transform: " << e.what());
                        }
                    } else {
                        NODELET_WARN_STREAM("Positional tracking failed: " << status);
                    }
                    mGrabThreadProfiler.measureEvent("Positional tracking");
                }

                // Publish IMU and magnetometer data
                if (mZedInfo.camera_model == sl::MODEL::ZED2i && mImuPub.getNumSubscribers()) {
                    sl::SensorsData sensorData;
                    mZed.getSensorsData(sensorData, sl::TIME_REFERENCE::CURRENT);

                    sensor_msgs::Imu imuMsg;
                    fillImuMessage(sensorData.imu, imuMsg);
                    imuMsg.header.frame_id = "zed2i_mag_frame";
                    imuMsg.header.stamp = ros::Time::now();
                    imuMsg.header.seq = mGrabUpdateTick;
                    mImuPub.publish(imuMsg);

                    sensor_msgs::MagneticField magMsg;
                    fillMagMessage(sensorData.magnetometer, magMsg);
                    magMsg.header.frame_id = "zed2i_mag_frame";
                    magMsg.header.stamp = ros::Time::now();
                    magMsg.header.seq = mGrabUpdateTick;
                    mMagPub.publish(magMsg);
                    mGrabThreadProfiler.measureEvent("Sensor data");
                }

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

    ZedNodelet::~ZedNodelet() {
        NODELET_INFO("ZED node shutting down");
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
