#include "zed_wrapper.hpp"

#include <chrono>

#include <nodelet/loader.h>
#include <ros/init.h>

#include <se3.hpp>

using namespace std::chrono_literals;

namespace mrover {

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

    void ZedNodelet::onInit() {
        try {
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();
            mPcPub = mNh.advertise<sensor_msgs::PointCloud2>("camera/left/points", 1);
            mImuPub = mNh.advertise<sensor_msgs::Imu>("imu", 1);
            mLeftCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>("camera/left/camera_info", 1);
            mRightCamInfoPub = mNh.advertise<sensor_msgs::CameraInfo>("camera/right/camera_info", 1);
            image_transport::ImageTransport it{mNh};
            mLeftImgPub = it.advertise("camera/left/image", 1);
            mRightImgPub = it.advertise("camera/right/image", 1);

            mNh.param<bool>("use_odom_frame", mUseOdom, false);

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
            mPnh.param("direct_tag_detection", mDirectTagDetection, false);
            std::string svoFile{};
            mPnh.param("svo_file", svoFile, {});
            mPnh.param("ues_builtin_visual_odom", mUseBuiltinPosTracking, false);

            if (imageWidth < 0 || imageHeight < 0) {
                throw std::invalid_argument("Invalid image dimensions");
            }
            if (mGrabTargetFps < 0) {
                throw std::invalid_argument("Invalid grab target framerate");
            }

            sl::InitParameters initParameters;
            if (!svoFile.empty()) {
                initParameters.input.setFromSVOFile(svoFile.c_str());
            }
            initParameters.camera_resolution = stringToZedEnum<sl::RESOLUTION>(grabResolutionString);
            initParameters.depth_mode = stringToZedEnum<sl::DEPTH_MODE>(depthModeString);
            initParameters.coordinate_units = sl::UNIT::METER;
            initParameters.sdk_verbose = true; // Log useful information
            initParameters.camera_fps = mGrabTargetFps;
            initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // Match ROS

            if (mZed.open(initParameters) != sl::ERROR_CODE::SUCCESS) {
                throw std::runtime_error("ZED failed to open");
            }

            if (mUseOdom && mUseBuiltinPosTracking) {
                sl::PositionalTrackingParameters positionalTrackingParameters;
                mZed.enablePositionalTracking(positionalTrackingParameters);
            }

            mGrabThread = std::thread(&ZedNodelet::grabUpdate, this);
            mPcThread = std::thread(&ZedNodelet::pointCloudUpdate, this);

        } catch (std::exception const& e) {
            NODELET_FATAL("Exception while starting: %s", e.what());
            ros::shutdown();
        }
    }

    /**
     * Only relevant if direct tag detection is true.
     * This handles loading a tag detection nodelet and passing it point clouds from the grab thread.
     */
    void ZedNodelet::pointCloudUpdate() {
        try {
            NODELET_INFO("Starting point cloud thread");

            if (mDirectTagDetection) {
                // TODO: ugly, this prevents OpenCV fast alloc from crashing
                std::this_thread::sleep_for(100ms);

                mTagDetectorNode = boost::make_shared<TagDetectorNodelet>();
                mTagDetectorNode->init("tag_detector", getRemappingArgs(), getMyArgv());

                // TODO: figure out why removing this causes a segfault
                cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            }

            while (ros::ok()) {
                mPcThreadProfiler.reset();

                {
                    std::unique_lock lock{mGrabMutex};
                    mGrabDone.wait(lock);
                }
                ros::Time grabTime{slTime2Ros(mZed.getTimestamp(sl::TIME_REFERENCE::IMAGE))};
                mPcThreadProfiler.addEpoch("Wait");

                if (mZed.retrieveImage(mLeftImageMat, sl::VIEW::LEFT, sl::MEM::CPU, mImageResolution) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to retrieve left image");
                if (mRightImgPub.getNumSubscribers())
                    if (mZed.retrieveImage(mRightImageMat, sl::VIEW::RIGHT, sl::MEM::CPU, mImageResolution) != sl::ERROR_CODE::SUCCESS)
                        throw std::runtime_error("ZED failed to retrieve right image");
                if (mZed.retrieveMeasure(mPointCloudXYZMat, sl::MEASURE::XYZ, sl::MEM::CPU, mImageResolution) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to retrieve point cloud");
                mPcThreadProfiler.addEpoch("Retrieve");

                fillPointCloudMessage(mPointCloudXYZMat, mLeftImageMat, mPointCloud);
                mPointCloud->header.seq = mPointCloudUpdateTick;
                mPointCloud->header.stamp = grabTime;
                mPointCloud->header.frame_id = "zed2i_left_camera_frame";
                mPcThreadProfiler.addEpoch("Fill Message");

                if (mDirectTagDetection)
                    mTagDetectorNode->pointCloudCallback(mPointCloud);
                mPcThreadProfiler.addEpoch("Direct Tag Detection");

                if (mPcPub.getNumSubscribers()) {
                    mPcPub.publish(mPointCloud);
                    if (mDirectTagDetection)
                        NODELET_WARN("Publishing defeats the purpose of direct tag detection");
                }
                mPcThreadProfiler.addEpoch("Point cloud publish");

                if (mLeftImgPub.getNumSubscribers()) {
                    fillImageMessage(mLeftImageMat, mLeftImgMsg);
                    mLeftImgMsg->header.frame_id = "zed2i_left_camera_frame";
                    mLeftImgMsg->header.stamp = grabTime;
                    mLeftImgMsg->header.seq = mPointCloudUpdateTick;
                    mLeftImgPub.publish(mLeftImgMsg);
                }
                if (mRightImgPub.getNumSubscribers()) {
                    fillImageMessage(mRightImageMat, mRightImgMsg);
                    mRightImgMsg->header.frame_id = "zed2i_right_camera_frame";
                    mRightImgMsg->header.stamp = grabTime;
                    mRightImgMsg->header.seq = mPointCloudUpdateTick;
                    mRightImgPub.publish(mRightImgMsg);
                }
                if (mLeftCamInfoPub.getNumSubscribers() || mRightCamInfoPub.getNumSubscribers()) {
                    sl::CalibrationParameters calibration = mZed.getCameraInformation(mImageResolution).camera_configuration.calibration_parameters;
                    fillCameraInfoMessages(calibration, mImageResolution, mLeftCamInfoMsg, mRightCamInfoMsg);
                    mLeftCamInfoMsg->header.frame_id = "zed2i_left_camera_frame";
                    mLeftCamInfoMsg->header.stamp = grabTime;
                    mLeftCamInfoMsg->header.seq = mPointCloudUpdateTick;
                    mRightCamInfoMsg->header.frame_id = "zed2i_right_camera_frame";
                    mRightCamInfoMsg->header.stamp = grabTime;
                    mRightCamInfoMsg->header.seq = mPointCloudUpdateTick;
                    mLeftCamInfoPub.publish(mLeftCamInfoMsg);
                    mRightCamInfoPub.publish(mRightCamInfoMsg);
                }

                mPcThreadProfiler.addEpoch("Image + camera info publish");

                mPointCloudUpdateTick++;
            }
            NODELET_INFO("Tag thread finished");

        } catch (std::exception const& e) {
            NODELET_FATAL("Exception while running tag thread: %s", e.what());
            ros::shutdown();
            std::exit(EXIT_FAILURE);
        }
    }

    void ZedNodelet::grabUpdate() {
        try {
            NODELET_INFO("Starting grab thread");
            while (ros::ok()) {
                mGrabThreadProfiler.reset();

                sl::RuntimeParameters runtimeParameters;
                runtimeParameters.confidence_threshold = mDepthConfidence;
                runtimeParameters.texture_confidence_threshold = mTextureConfidence;

                if (mZed.grab(runtimeParameters) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to grab");
                mGrabThreadProfiler.addEpoch("Grab");

                {
                    std::unique_lock lock{mGrabMutex};
                    mGrabDone.notify_all();
                }

                if (mUseOdom && mUseBuiltinPosTracking) {
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
                    mGrabThreadProfiler.addEpoch("Positional tracking");
                }

                if (mImuPub.getNumSubscribers()) {
                    sl::SensorsData sensorData;
                    mZed.getSensorsData(sensorData, sl::TIME_REFERENCE::CURRENT);

                    sensor_msgs::Imu imuMsg;
                    fillImuMessage(sensorData.imu, imuMsg);
                    imuMsg.header.frame_id = "zed2i_imu_frame";
                    imuMsg.header.stamp = ros::Time::now();
                    imuMsg.header.seq = mGrabUpdateTick;
                    mImuPub.publish(imuMsg);
                }
                mGrabThreadProfiler.addEpoch("Sensor data");

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
        mPcThread.join();
        mGrabThread.join();
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

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::ZedNodelet, nodelet::Nodelet)
