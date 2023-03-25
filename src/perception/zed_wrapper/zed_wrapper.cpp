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
            mPnh.param("grab_target_fps", mGrabTargetFps, 50);
            mPnh.param("image_width", mImageWidth, 1280);
            mPnh.param("image_height", mImageHeight, 720);
            mPnh.param("direct_tag_detection", mDirectTagDetection, false);
            std::string svoFile{};
            mPnh.param("svo_file", svoFile, {});

            if (mImageWidth < 0 || mImageHeight < 0 || mImageHeight * 16 / 9 != mImageWidth) {
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
            initParameters.depth_mode = sl::DEPTH_MODE::QUALITY;
            initParameters.coordinate_units = sl::UNIT::METER;
            initParameters.sdk_verbose = true;
            initParameters.camera_fps = mGrabTargetFps;
            initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;

            if (mZed.open(initParameters) != sl::ERROR_CODE::SUCCESS) {
                throw std::runtime_error("ZED failed to open");
            }

            sl::PositionalTrackingParameters positionalTrackingParameters;
            mZed.enablePositionalTracking(positionalTrackingParameters);

            mGrabThread = std::thread(&ZedNodelet::grabUpdate, this);
            mTagThread = std::thread(&ZedNodelet::tagUpdate, this);
            //            if (optimizeTagDetection) {
            //                ROS_INFO("Loading tag detector nodelet");
            //                pluginlib::ClassLoader<TagDetectorNodelet> tagDetectorLoader{"mrover", "nodelet::Nodelet"};
            //                mTagDetectorNode = tagDetectorLoader.createInstance("mrover::TagDetectorNodelet");
            //            }
        } catch (std::exception const& e) {
            ROS_FATAL("Exception while starting: %s", e.what());
            ros::shutdown();
        }
    }

    void ZedNodelet::tagUpdate() {
        try {
            if (!mDirectTagDetection) return;

            ROS_INFO("Starting tag thread");

            // TODO: ugly, this prevents OpenCV fast alloc from crashing
            std::this_thread::sleep_for(100ms);

            mTagDetectorNode = boost::make_shared<TagDetectorNodelet>();
            mTagDetectorNode->init("tag_detector", getRemappingArgs(), getMyArgv());

            // TODO: figure out why removing this causes a segfault
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

            while (ros::ok()) {
                std::unique_lock lock{mSwapPcMutex};
                mGrabDone.wait(lock, [this] { return mIsGrabDone; });

                mTagDetectorNode->pointCloudCallback(mTagPointCloud);
            }
            ROS_INFO("Tag thread finished");

        } catch (std::exception const& e) {
            ROS_FATAL("Exception while running tag thread: %s", e.what());
            ros::shutdown();
            std::exit(EXIT_FAILURE);
        }
    }

    void ZedNodelet::grabUpdate() {
        try {
            ROS_INFO("Starting grab thread");
            while (ros::ok()) {
                hr_clock::time_point update_start = hr_clock::now();

                sl::RuntimeParameters runtimeParameters;
                runtimeParameters.confidence_threshold = 80;
                runtimeParameters.texture_confidence_threshold = 80;

                sl::Resolution processResolution(mImageWidth, mImageHeight);

                if (mZed.grab(runtimeParameters) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to grab");
                if (mZed.retrieveImage(mLeftImageMat, sl::VIEW::LEFT, sl::MEM::CPU, processResolution) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to retrieve image");
                if (mZed.retrieveMeasure(mPointCloudXYZMat, sl::MEASURE::XYZ, sl::MEM::CPU, processResolution) != sl::ERROR_CODE::SUCCESS)
                    throw std::runtime_error("ZED failed to retrieve point cloud");
                //            mZed.retrieveMeasure(mPointCloudNormalMat, sl::MEASURE::NORMALS, sl::MEM::CPU, mImageResolution);
                hr_clock::duration grab_time = hr_clock::now() - update_start;

                mIsGrabDone = false;
                fillPointCloudMessage(mPointCloudXYZMat, mLeftImageMat, mGrabPointCloud, mUpdateTick);
                hr_clock::duration to_msg_time = hr_clock::now() - update_start - grab_time;

                if (mPcPub.getNumSubscribers()) {
                    mPcPub.publish(mGrabPointCloud);
                    if (mDirectTagDetection) {
                        ROS_WARN("Publishing defeats the purpose of direct tag detection");
                    }
                }
                if (mDirectTagDetection && mSwapPcMutex.try_lock()) {
                    mTagPointCloud.swap(mGrabPointCloud);
                    mIsGrabDone = true;
                    mSwapPcMutex.unlock();
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
                    ROS_DEBUG_STREAM("Position: " << translation.x << ", " << translation.y << ", " << translation.z);
                    ROS_DEBUG_STREAM("Orientation: " << orientation.w << ", " << orientation.x << ", " << orientation.y << ", " << orientation.z);
                    try {
                        SE3 leftCameraInOdom{{translation.x, translation.y, translation.z},
                                             Eigen::Quaterniond{orientation.w, orientation.x, orientation.y, orientation.z}.normalized()};
                        SE3 leftCameraInBaseLink = SE3::fromTfTree(mTfBuffer, "base_link", "zed2i_left_camera_frame");
                        SE3 baseLinkInOdom = leftCameraInBaseLink * leftCameraInOdom;
                        SE3::pushToTfTree(mTfBroadcaster, "base_link", "odom", baseLinkInOdom);
                    } catch (tf2::TransformException& e) {
                        ROS_WARN_STREAM("Failed to get transform: " << e.what());
                    }
                } else {
                    ROS_WARN_STREAM("Positional tracking failed: " << status);
                }

                hr_clock::duration update_duration = hr_clock::now() - update_start;
                if (mUpdateTick % 60 == 0) {
                    ROS_INFO_STREAM("[" << std::hash<std::thread::id>{}(std::this_thread::get_id()) << "] ZED Total: " << std::chrono::duration_cast<std::chrono::milliseconds>(update_duration).count() << "ms");
                    ROS_INFO_STREAM("\tGrab: " << std::chrono::duration_cast<std::chrono::milliseconds>(grab_time).count() << "ms");
                    ROS_INFO_STREAM("\tTo msg: " << std::chrono::duration_cast<std::chrono::milliseconds>(to_msg_time).count() << "ms");
                    ROS_INFO_STREAM("\tPublish: " << std::chrono::duration_cast<std::chrono::milliseconds>(publish_time).count() << "ms");
                }

                if (mImuPub.getNumSubscribers()) {
                    sl::SensorsData sensorData;
                    mZed.getSensorsData(sensorData, sl::TIME_REFERENCE::CURRENT);

                    sensor_msgs::Imu imuMsg;
                    fillImuMessage(sensorData.imu, imuMsg, mUpdateTick);
                    mImuPub.publish(imuMsg);
                }

                mUpdateTick++;
            }

            mZed.close();
            ROS_INFO("Grab thread finished");

        } catch (std::exception const& e) {
            ROS_FATAL("Exception while running grab thread: %s", e.what());
            mZed.close();
            ros::shutdown();
            std::exit(EXIT_FAILURE);
        }
    }

    ZedNodelet::~ZedNodelet() {
        ROS_INFO("ZED node shutting down");
        mTagThread.join();
        mGrabThread.join();
    }

} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_wrapper");

    // Start the ZED Nodelet
    nodelet::Loader nodelet;
    nodelet::M_string remap(ros::names::getRemappings());
    nodelet::V_string nargv;
    nodelet.load(ros::this_node::getName(), "mrover/ZedNodelet", remap, nargv);

    ros::spin();

    return EXIT_SUCCESS;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::ZedNodelet, nodelet::Nodelet)
