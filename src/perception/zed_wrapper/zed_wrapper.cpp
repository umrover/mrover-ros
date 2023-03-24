#include "zed_wrapper.hpp"

#include <algorithm>
#include <chrono>
#include <execution>

#include <ros/init.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <se3.hpp>

using namespace std::chrono_literals;
using hr_clock = std::chrono::high_resolution_clock;

struct Point {
    float x, y, z;
    uint8_t r, g, b, a;
    float normal_x, normal_y, normal_z;
    float curvature;
} __attribute__((packed));

ZedNode::ZedNode(ros::NodeHandle const& nh, ros::NodeHandle const& pnh)
    : mNh{nh}, mPnh{pnh}, mTfListener{mTfBuffer}, mIt{mNh},
      mPcPub{mNh.advertise<sensor_msgs::PointCloud2>("/camera/left/points", 1)},
      mLeftImgPub{mIt.advertise("/camera/left/image", 1)} {

    mNh.param("grab_resolution", mResolution, static_cast<std::underlying_type_t<sl::RESOLUTION>>(sl::RESOLUTION::HD720));
    mNh.param("grab_target_fps", mGrabTargetFps, 50);
    mNh.param("image_width", mImageWidth, 1280);
    mNh.param("image_height", mImageHeight, 720);

    sl::InitParameters initParameters;
    initParameters.camera_resolution = static_cast<sl::RESOLUTION>(mResolution);
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

    //    mTagDetectorNode = std::make_unique<TagDetectorNode>(mNh, mPnh);
}

void ZedNode::tagUpdate() {
    while (ros::ok() && mTagDetectorNode) {
        std::unique_lock lock{mSwapPcMutex};
        mGrabDone.wait(lock, [this] { return mIsGrabDone; });

        mTagDetectorNode->pointCloudCallback(mTagPointCloud);
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

void ZedNode::grabUpdate() {
    while (ros::ok() && mZed.isOpened()) {
        hr_clock::time_point update_start = hr_clock::now();

        sl::RuntimeParameters runtimeParameters;
        runtimeParameters.confidence_threshold = 80;
        runtimeParameters.texture_confidence_threshold = 80;

        sl::Resolution processResolution(mImageWidth, mImageHeight);

        if (mZed.grab(runtimeParameters) == sl::ERROR_CODE::SUCCESS) {
            mZed.retrieveImage(mLeftImageMat, sl::VIEW::LEFT, sl::MEM::CPU, processResolution);
            mZed.retrieveMeasure(mPointCloudXYZMat, sl::MEASURE::XYZ, sl::MEM::CPU, processResolution);
            //            mZed.retrieveMeasure(mPointCloudNormalMat, sl::MEASURE::NORMALS, sl::MEM::CPU, mImageResolution);
            hr_clock::duration grab_time = hr_clock::now() - update_start;

            fillPointCloudMessage(mPointCloudXYZMat, mLeftImageMat, mGrabPointCloud, mUpdateTick);
            if (mSwapPcMutex.try_lock()) {
                std::swap(mTagPointCloud, mGrabPointCloud);
                mIsGrabDone = true;
                mSwapPcMutex.unlock();
                mGrabDone.notify_one();
            }
            hr_clock::duration to_msg_time = hr_clock::now() - update_start - grab_time;

            if (mPcPub.getNumSubscribers()) mPcPub.publish(mGrabPointCloud);
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
                ROS_INFO_STREAM("ZED Total: " << std::chrono::duration_cast<std::chrono::milliseconds>(update_duration).count() << "ms");
                ROS_INFO_STREAM("\tGrab: " << std::chrono::duration_cast<std::chrono::milliseconds>(grab_time).count() << "ms");
                ROS_INFO_STREAM("\tTo msg: " << std::chrono::duration_cast<std::chrono::milliseconds>(to_msg_time).count() << "ms");
                ROS_INFO_STREAM("\tPublish: " << std::chrono::duration_cast<std::chrono::milliseconds>(publish_time).count() << "ms");
            }

            mUpdateTick++;
        } else {
            throw std::runtime_error("ZED failed to grab");
        }
    }
}

ZedNode::~ZedNode() {
    mZed.close();
    mTagThread.join();
    mGrabThread.join();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_wrapper");

    [[maybe_unused]] auto node = std::make_shared<ZedNode>();

    ros::spin();

    return EXIT_SUCCESS;
}
