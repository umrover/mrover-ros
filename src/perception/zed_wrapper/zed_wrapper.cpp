#include "zed_wrapper.hpp"

#include <algorithm>
#include <chrono>
#include <execution>

#include <ros/init.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <se3.hpp>

constexpr float DEG2RAD = M_PI / 180.0f;

using namespace std::chrono_literals;
using hr_clock = std::chrono::high_resolution_clock;

namespace mrover {

    struct Point {
        float x, y, z;
        uint8_t r, g, b, a;
        float normal_x, normal_y, normal_z;
        float curvature;
    } __attribute__((packed));

    ZedNode::ZedNode(ros::NodeHandle const& nh, ros::NodeHandle const& pnh)
        : mNh{nh}, mPnh{pnh}, mTfListener{mTfBuffer}, mIt{mNh},
          mPcPub{mNh.advertise<sensor_msgs::PointCloud2>("camera/left/points", 1)},
          mImuPub{mNh.advertise<sensor_msgs::Imu>("imu", 1)},
          mLeftImgPub{mIt.advertise("camera/left/image", 1)} {

        try {
            int resolution{};
            mPnh.param("grab_resolution", resolution, static_cast<std::underlying_type_t<sl::RESOLUTION>>(sl::RESOLUTION::HD720));
            mPnh.param("grab_target_fps", mGrabTargetFps, 50);
            mPnh.param("image_width", mImageWidth, 1280);
            mPnh.param("image_height", mImageHeight, 720);
            mPnh.param("optimize_tag_detection", mDirectTagDetection, true);
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

            mGrabThread = std::thread(&ZedNode::grabUpdate, this);
            mTagThread = std::thread(&ZedNode::tagUpdate, this);
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

    void fillPointCloudMessage(sl::Mat& xyz, sl::Mat& bgr, sensor_msgs::PointCloud2Ptr const& msg, size_t tick) {
        if (xyz.getWidth() != bgr.getWidth() || xyz.getHeight() != bgr.getHeight()) {
            throw std::invalid_argument("XYZ and RGB images must be the same size");
        }

        auto imagePtr = bgr.getPtr<sl::uchar4>();
        auto* pointCloudPtr = xyz.getPtr<sl::float4>();
        //            auto* pointCloudNormalPtr = mPointCloudNormalMat.getPtr<sl::float4>();
        msg->header.frame_id = "zed2i_left_camera_frame";
        msg->header.seq = tick;
        msg->header.stamp = ros::Time::now();
        msg->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        msg->is_dense = false;
        msg->height = xyz.getHeight();
        msg->width = xyz.getWidth();
        sensor_msgs::PointCloud2Modifier modifier{*msg};
        modifier.setPointCloud2Fields(
                8,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "rgb", 1, sensor_msgs::PointField::FLOAT32,
                "normal_x", 1, sensor_msgs::PointField::FLOAT32,
                "normal_y", 1, sensor_msgs::PointField::FLOAT32,
                "normal_z", 1, sensor_msgs::PointField::FLOAT32,
                "curvature", 1, sensor_msgs::PointField::FLOAT32);
        auto* pointPtr = reinterpret_cast<Point*>(msg->data.data());
        size_t size = msg->width * msg->height;
        std::for_each(std::execution::par_unseq, pointPtr, pointPtr + size, [&](Point& point) {
            size_t i = &point - pointPtr;
            point.x = pointCloudPtr[i].x;
            point.y = pointCloudPtr[i].y;
            point.z = pointCloudPtr[i].z;
            point.r = imagePtr[i].r;
            point.g = imagePtr[i].g;
            point.b = imagePtr[i].b;
        });
    }

    void fillImageMessage(sl::Mat& bgr, sensor_msgs::Image& msg, size_t tick) {
        msg.header.frame_id = "zed2i_left_camera_frame";
        msg.header.seq = tick;
        msg.header.stamp = ros::Time::now();
        msg.height = bgr.getHeight();
        msg.width = bgr.getWidth();
        msg.encoding = sensor_msgs::image_encodings::BGRA8;
        msg.step = bgr.getStepBytes();
        msg.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        auto* bgrPtr = bgr.getPtr<sl::uchar1>();
        size_t size = msg.step * msg.height;
        msg.data.resize(size);
        std::memcpy(msg.data.data(), bgrPtr, size);
    }

    void ZedNode::tagUpdate() {
        try {
            if (!mDirectTagDetection) return;

            ROS_INFO("Starting tag thread");
            mTagDetectorNode = boost::make_shared<TagDetectorNode>(mNh, mPnh, true);

            std::this_thread::sleep_for(500ms);

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

    void ZedNode::grabUpdate() {
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
                    imuMsg.header.stamp = ros::Time::now();
                    imuMsg.header.seq = mUpdateTick;
                    imuMsg.header.frame_id = "zed2i_imu_frame";
                    imuMsg.orientation.x = sensorData.imu.pose.getOrientation().x;
                    imuMsg.orientation.y = sensorData.imu.pose.getOrientation().y;
                    imuMsg.orientation.z = sensorData.imu.pose.getOrientation().z;
                    imuMsg.orientation.w = sensorData.imu.pose.getOrientation().w;
                    for (int i = 0; i < 3; ++i)
                        for (int j = 0; j < 3; ++j)
                            imuMsg.orientation_covariance[i * 3 + j] = sensorData.imu.pose_covariance(i, j) * DEG2RAD * DEG2RAD;
                    imuMsg.angular_velocity.x = sensorData.imu.angular_velocity.x * DEG2RAD;
                    imuMsg.angular_velocity.y = sensorData.imu.angular_velocity.y * DEG2RAD;
                    imuMsg.angular_velocity.z = sensorData.imu.angular_velocity.z * DEG2RAD;
                    for (int i = 0; i < 3; ++i)
                        for (int j = 0; j < 3; ++j)
                            imuMsg.angular_velocity_covariance[i * 3 + j] = sensorData.imu.angular_velocity_covariance(i, j) * DEG2RAD * DEG2RAD;
                    imuMsg.linear_acceleration.x = sensorData.imu.linear_acceleration.x;
                    imuMsg.linear_acceleration.y = sensorData.imu.linear_acceleration.y;
                    imuMsg.linear_acceleration.z = sensorData.imu.linear_acceleration.z;
                    for (int i = 0; i < 3; ++i)
                        for (int j = 0; j < 3; ++j)
                            imuMsg.linear_acceleration_covariance[i * 3 + j] = sensorData.imu.linear_acceleration_covariance(i, j);

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

    ZedNode::~ZedNode() {
        ROS_INFO("ZED node shutting down");
        mTagThread.join();
        mGrabThread.join();
    }

    void ZedNodelet::onInit() {
        dtl = boost::make_shared<ZedNode>(getMTNodeHandle(), getMTPrivateNodeHandle());
    }

} // namespace mrover

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_wrapper");

    [[maybe_unused]] auto node = std::make_shared<mrover::ZedNode>();

    ros::spin();

    return EXIT_SUCCESS;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mrover::ZedNodelet, nodelet::Nodelet)
