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

    sl::InitParameters initParameters;
    initParameters.camera_resolution = sl::RESOLUTION::HD720;
    initParameters.depth_mode = sl::DEPTH_MODE::QUALITY;
    initParameters.coordinate_units = sl::UNIT::METER;
    initParameters.sdk_verbose = true;
    initParameters.camera_fps = 50;
    initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;

    if (mZed.open(initParameters) != sl::ERROR_CODE::SUCCESS) {
        throw std::runtime_error("ZED failed to open");
    }

    sl::PositionalTrackingParameters positionalTrackingParameters;
    mZed.enablePositionalTracking(positionalTrackingParameters);

    mUpdateThread = std::thread(&ZedNode::update, this);

    //    mTagDetectorNode = std::make_unique<TagDetectorNode>(mNh, mPnh);
}

void ZedNode::update() {
    while (ros::ok() && mZed.isOpened()) {
        hr_clock::time_point update_start = hr_clock::now();

        sl::RuntimeParameters runtimeParameters;
        runtimeParameters.confidence_threshold = 80;
        runtimeParameters.texture_confidence_threshold = 80;

        if (mZed.grab(runtimeParameters) == sl::ERROR_CODE::SUCCESS) {
            mZed.retrieveImage(mImageMat, sl::VIEW::LEFT, sl::MEM::CPU, mImageResolution);
            mZed.retrieveMeasure(mPointCloudXYZMat, sl::MEASURE::XYZ, sl::MEM::CPU, mImageResolution);
            hr_clock::duration grab_time = hr_clock::now() - update_start;
            //            mZed.retrieveMeasure(mPointCloudNormalMat, sl::MEASURE::NORMALS, sl::MEM::CPU, mImageResolution);

            auto imagePtr = mImageMat.getPtr<sl::uchar4>();
            auto* pointCloudPtr = mPointCloudXYZMat.getPtr<sl::float4>();
            //            auto* pointCloudNormalPtr = mPointCloudNormalMat.getPtr<sl::float4>();

            mPointCloudMsg.header.frame_id = "zed2i_left_camera_frame";
            mPointCloudMsg.header.seq = mUpdateTick;
            mPointCloudMsg.header.stamp = ros::Time::now();
            mPointCloudMsg.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
            mPointCloudMsg.is_dense = false;
            mPointCloudMsg.height = mImageResolution.height;
            mPointCloudMsg.width = mImageResolution.width;
            sensor_msgs::PointCloud2Modifier modifier{mPointCloudMsg};
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
            auto* pointPtr = reinterpret_cast<Point*>(mPointCloudMsg.data.data());
            std::for_each(std::execution::par, pointPtr, pointPtr + mImageResolution.area(), [&](Point& point) {
                size_t i = &point - pointPtr;
                point.x = pointCloudPtr[i].x;
                point.y = pointCloudPtr[i].y;
                point.z = pointCloudPtr[i].z;
                point.r = imagePtr[i].r;
                point.g = imagePtr[i].g;
                point.b = imagePtr[i].b;
            });
            //            for (size_t i = 0; i < mImageResolution.area(); ++i) {
            //                pointPtr[i].x = pointCloudPtr[i].x;
            //                pointPtr[i].y = pointCloudPtr[i].y;
            //                pointPtr[i].z = pointCloudPtr[i].z;
            //                pointPtr[i].r = imagePtr[i].r;
            //                pointPtr[i].g = imagePtr[i].g;
            //                pointPtr[i].b = imagePtr[i].b;
            //            }
            hr_clock::duration to_msg_time = hr_clock::now() - update_start - grab_time;
            if (mTagDetectorNode) {
                sensor_msgs::PointCloud2ConstPtr ptr;
                ptr.reset(&mPointCloudMsg, [](auto*) {});
                mTagDetectorNode->pointCloudCallback(ptr);
            } else {
                mPcPub.publish(mPointCloudMsg);
            }
            hr_clock::duration publish_time = hr_clock::now() - update_start - grab_time - to_msg_time;

            if (mLeftImgPub.getNumSubscribers()) {
                mLeftImgMsg.header.frame_id = "zed2i_left_camera_frame";
                mLeftImgMsg.header.seq = mUpdateTick;
                mLeftImgMsg.header.stamp = ros::Time::now();
                mLeftImgMsg.height = mImageMat.getHeight();
                mLeftImgMsg.width = mImageMat.getWidth();
                mLeftImgMsg.encoding = sensor_msgs::image_encodings::BGRA8;
                mLeftImgMsg.step = mImageMat.getStepBytes();
                mLeftImgMsg.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
                auto* data = mImageMat.getPtr<sl::uchar1>();
                size_t size = mLeftImgMsg.step * mLeftImgMsg.height;
                mLeftImgMsg.data.resize(size);
                std::memcpy(mLeftImgMsg.data.data(), data, size);
                mLeftImgPub.publish(mLeftImgMsg);
            }

            sl::Pose pose;
            sl::POSITIONAL_TRACKING_STATE status = mZed.getPosition(pose);
            if (status == sl::POSITIONAL_TRACKING_STATE::OK) {
                sl::Translation const& translation = pose.getTranslation();
                sl::Orientation const& orientation = pose.getOrientation();
                ROS_DEBUG_STREAM("Position: " << translation.x << ", " << translation.y << ", " << translation.z);
                ROS_DEBUG_STREAM("Orientation: " << orientation.w << ", " << orientation.x << ", " << orientation.y << ", " << orientation.z);
                Eigen::Quaterniond q{orientation.w, orientation.x, orientation.y, orientation.z};
                try {
                    SE3 leftCameraInOdom{{translation.x, translation.y, translation.z}, q.normalized()};
                    SE3 leftCameraInBaseLink = SE3::fromTfTree(mTfBuffer, "base_link", "zed2i_left_camera_frame");
                    SE3 baseLinkInOdom = leftCameraInBaseLink * leftCameraInOdom;
                    SE3::pushToTfTree(mTfBroadcaster, "base_link", "odom", baseLinkInOdom);
                } catch (tf2::TransformException& e) {
                    ROS_WARN_STREAM("Failed to get transform: " << e.what());
                }
            } else {
                ROS_WARN_STREAM("Positional tracking failed: " << status);
            }

            mUpdateTick++;

            hr_clock::duration update_duration = hr_clock::now() - update_start;

            if (mUpdateTick % 60 == 0) {
                ROS_INFO_STREAM("Total: " << std::chrono::duration_cast<std::chrono::milliseconds>(update_duration).count() << "ms");
                ROS_INFO_STREAM("\tGrab: " << std::chrono::duration_cast<std::chrono::milliseconds>(grab_time).count() << "ms");
                ROS_INFO_STREAM("\tTo msg: " << std::chrono::duration_cast<std::chrono::milliseconds>(to_msg_time).count() << "ms");
                ROS_INFO_STREAM("\tPublish: " << std::chrono::duration_cast<std::chrono::milliseconds>(publish_time).count() << "ms");
            }
        } else {
            throw std::runtime_error("ZED failed to grab");
        }
    }
}

ZedNode::~ZedNode() {
    mZed.close();
    mUpdateThread.join();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "zed_wrapper");

    [[maybe_unused]] auto node = std::make_shared<ZedNode>();

    ros::spin();

    return EXIT_SUCCESS;
}
