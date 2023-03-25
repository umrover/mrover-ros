#include "zed_wrapper.hpp"

#include <chrono>

#include <nodelet/loader.h>
#include <ros/init.h>

#include <se3.hpp>

using namespace std::chrono_literals;
using hr_clock = std::chrono::high_resolution_clock;

namespace mrover {

    void ZedNodelet::onInit() {
        try {
            mNh = getMTNodeHandle();
            mPnh = getMTPrivateNodeHandle();
            mPcPub = mNh.advertise<sensor_msgs::PointCloud2>("camera/left/points", 1);
            mImuPub = mNh.advertise<sensor_msgs::Imu>("imu", 1);
            image_transport::ImageTransport it{mNh};
            mLeftImgPub = it.advertise("camera/left/image", 1);

            int resolution{};
            mPnh.param("grab_resolution", resolution, static_cast<std::underlying_type_t<sl::RESOLUTION>>(sl::RESOLUTION::HD720));
            int depthMode{};
            mPnh.param("depth_mode", depthMode, static_cast<std::underlying_type_t<sl::RESOLUTION>>(sl::DEPTH_MODE::PERFORMANCE));
            mPnh.param("grab_target_fps", mGrabTargetFps, 50);
            mPnh.param("image_width", mImageWidth, 1280);
            mPnh.param("image_height", mImageHeight, 720);
            mPnh.param("depth_confidence", mDepthConfidence, 70);
            mPnh.param("texture_confidence", mTextureConfidence, 70);
            mPnh.param("direct_tag_detection", mDirectTagDetection, false);
            std::string svoFile{};
            mPnh.param("svo_file", svoFile, {});


            if (mImageWidth < 0 || mImageHeight < 0) {
                throw std::invalid_argument("Invalid image dimensions");
            }
            if (mGrabTargetFps < 0) {
                throw std::invalid_argument("Invalid grab target framerate");
            }

            sl::InitParameters initParameters;
            if (!svoFile.empty()) {
                initParameters.input.setFromSVOFile(svoFile.c_str());
            }
            initParameters.camera_resolution = static_cast<sl::RESOLUTION>(resolution);
            initParameters.depth_mode = static_cast<sl::DEPTH_MODE>(depthMode);
            initParameters.coordinate_units = sl::UNIT::METER;
            initParameters.sdk_verbose = true; // Log useful information
            initParameters.camera_fps = mGrabTargetFps;
            initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD; // Match ROS

            if (mZed.open(initParameters) != sl::ERROR_CODE::SUCCESS) {
                throw std::runtime_error("ZED failed to open");
            }

            sl::PositionalTrackingParameters positionalTrackingParameters;
            mZed.enablePositionalTracking(positionalTrackingParameters);

            mGrabThread = std::thread(&ZedNodelet::grabUpdate, this);
            mTagThread = std::thread(&ZedNodelet::tagUpdate, this);

        } catch (std::exception const& e) {
            NODELET_FATAL("Exception while starting: %s", e.what());
            ros::shutdown();
        }
    }

    /**
     * Only relevant if direct tag detection is true.
     * This handles loading a tag detection nodelet and passing it point clouds from the grab thread.
     */
    void ZedNodelet::tagUpdate() {
        try {
            if (!mDirectTagDetection) return;

            NODELET_INFO("Starting tag thread");

            // TODO: ugly, this prevents OpenCV fast alloc from crashing
            std::this_thread::sleep_for(100ms);

            mTagDetectorNode = boost::make_shared<TagDetectorNodelet>();
            mTagDetectorNode->init("tag_detector", getRemappingArgs(), getMyArgv());

            // TODO: figure out why removing this causes a segfault
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

            while (ros::ok()) {
                std::unique_lock lock{mPcSwapMutex};
                mGrabDone.wait(lock, [this] { return mIsPcSwapReady; });

                mTagDetectorNode->pointCloudCallback(mTagPointCloud);
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
                hr_clock::time_point update_start = hr_clock::now();

                sl::RuntimeParameters runtimeParameters;
                runtimeParameters.confidence_threshold = mDepthConfidence;
                runtimeParameters.texture_confidence_threshold = mTextureConfidence;

                sl::Resolution processResolution(mImageWidth, mImageHeight);

                if (mZed.grab(runtimeParameters) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to grab");
                if (mZed.retrieveImage(mLeftImageMat, sl::VIEW::LEFT, sl::MEM::CPU, processResolution) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to retrieve image");
                if (mZed.retrieveMeasure(mPointCloudXYZMat, sl::MEASURE::XYZ, sl::MEM::CPU, processResolution) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to retrieve point cloud");
                //            mZed.retrieveMeasure(mPointCloudNormalMat, sl::MEASURE::NORMALS, sl::MEM::CPU, mImageResolution);
                hr_clock::duration grab_time = hr_clock::now() - update_start;

                mIsPcSwapReady = false;
                fillPointCloudMessage(mPointCloudXYZMat, mLeftImageMat, mGrabPointCloud, mUpdateTick);
                hr_clock::duration to_msg_time = hr_clock::now() - update_start - grab_time;

                if (mPcPub.getNumSubscribers()) {
                    mPcPub.publish(mGrabPointCloud);
                    if (mDirectTagDetection) {
                        NODELET_WARN("Publishing defeats the purpose of direct tag detection");
                    }
                }
                // If the tag detection thread is currently processing a point cloud, skip instead of blocking
                if (mDirectTagDetection && mPcSwapMutex.try_lock()) {
                    mTagPointCloud.swap(mGrabPointCloud);
                    mIsPcSwapReady = true;
                    mPcSwapMutex.unlock();
                    mGrabDone.notify_one();
                }
                hr_clock::duration publish_time = hr_clock::now() - update_start - grab_time - to_msg_time;

                if (mLeftImgPub.getNumSubscribers()) {
                    fillImageMessage(mLeftImageMat, mLeftImgMsg, mUpdateTick);
                    mLeftImgPub.publish(mLeftImgMsg);
                }

                sl::Pose pose;
                sl::POSITIONAL_TRACKING_STATE status = mZed.getPosition(pose);
                if (status == sl::POSITIONAL_TRACKING_STATE::OK) {
                    sl::Translation const& translation = pose.getTranslation();
                    sl::Orientation const& orientation = pose.getOrientation();
                    NODELET_DEBUG_STREAM("Position: " << translation.x << ", " << translation.y << ", " << translation.z);
                    NODELET_DEBUG_STREAM("Orientation: " << orientation.w << ", " << orientation.x << ", " << orientation.y << ", " << orientation.z);
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

                if (mImuPub.getNumSubscribers()) {
                    sl::SensorsData sensorData;
                    mZed.getSensorsData(sensorData, sl::TIME_REFERENCE::CURRENT);

                    sensor_msgs::Imu imuMsg;
                    fillImuMessage(sensorData.imu, imuMsg, mUpdateTick);
                    mImuPub.publish(imuMsg);
                }

                hr_clock::duration update_duration = hr_clock::now() - update_start;
                if (mUpdateTick % 60 == 0) {
                    NODELET_INFO_STREAM("[" << std::hash<std::thread::id>{}(std::this_thread::get_id()) << "] ZED Total: " << std::chrono::duration_cast<std::chrono::milliseconds>(update_duration).count() << "ms");
                    NODELET_INFO_STREAM("\tGrab: " << std::chrono::duration_cast<std::chrono::milliseconds>(grab_time).count() << "ms");
                    NODELET_INFO_STREAM("\tTo msg: " << std::chrono::duration_cast<std::chrono::milliseconds>(to_msg_time).count() << "ms");
                    NODELET_INFO_STREAM("\tPublish: " << std::chrono::duration_cast<std::chrono::milliseconds>(publish_time).count() << "ms");
                }

                mUpdateTick++;
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
        mTagThread.join();
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
