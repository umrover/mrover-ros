#include "zed_wrapper.hpp"

#include <algorithm>
#include <execution>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sl/Camera.hpp>
#include <stdexcept>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

constexpr float DEG2RAD = M_PI / 180.0f;

namespace mrover {

    struct Point {
        float x, y, z;
        uint8_t r, g, b, a;
        float normal_x, normal_y, normal_z;
        float curvature;
    } __attribute__((packed));

    ros::Time slTime2Ros(sl::Timestamp t) {
        auto sec = static_cast<uint32_t>(t.getNanoseconds() / 1000000000);
        auto nsec = static_cast<uint32_t>(t.getNanoseconds() % 1000000000);
        return {sec, nsec};
    }

    void fillPointCloudMessage(sl::Mat& xyz, sl::Mat& bgr, sensor_msgs::PointCloud2Ptr const& msg) {
        if (xyz.getWidth() != bgr.getWidth() || xyz.getHeight() != bgr.getHeight()) {
            throw std::invalid_argument("XYZ and RGB images must be the same size");
        }

        auto imagePtr = bgr.getPtr<sl::uchar4>();
        auto* pointCloudPtr = xyz.getPtr<sl::float4>();
        //            auto* pointCloudNormalPtr = mPointCloudNormalMat.getPtr<sl::float4>();
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

    void fillImageMessage(sl::Mat& bgr, sensor_msgs::ImagePtr const& msg) {
        msg->height = bgr.getHeight();
        msg->width = bgr.getWidth();
        msg->encoding = sensor_msgs::image_encodings::BGRA8;
        msg->step = bgr.getStepBytes();
        msg->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        auto* bgrPtr = bgr.getPtr<sl::uchar1>();
        size_t size = msg->step * msg->height;
        msg->data.resize(size);
        std::copy(std::execution::par_unseq, bgrPtr, bgrPtr + size, msg->data.begin());
    }

    void fillImuMessage(sl::SensorsData::IMUData& imuData, sensor_msgs::Imu& msg) {
        msg.header.stamp = ros::Time::now();
        msg.orientation.x = imuData.pose.getOrientation().x;
        msg.orientation.y = imuData.pose.getOrientation().y;
        msg.orientation.z = imuData.pose.getOrientation().z;
        msg.orientation.w = imuData.pose.getOrientation().w;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                msg.orientation_covariance[i * 3 + j] = imuData.pose_covariance(i, j) * DEG2RAD * DEG2RAD;
        msg.angular_velocity.x = imuData.angular_velocity.x * DEG2RAD;
        msg.angular_velocity.y = imuData.angular_velocity.y * DEG2RAD;
        msg.angular_velocity.z = imuData.angular_velocity.z * DEG2RAD;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                msg.angular_velocity_covariance[i * 3 + j] = imuData.angular_velocity_covariance(i, j) * DEG2RAD * DEG2RAD;
        msg.linear_acceleration.x = imuData.linear_acceleration.x;
        msg.linear_acceleration.y = imuData.linear_acceleration.y;
        msg.linear_acceleration.z = imuData.linear_acceleration.z;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                msg.linear_acceleration_covariance[i * 3 + j] = imuData.linear_acceleration_covariance(i, j);
    }

    void fillCameraInfoMessages(sl::CalibrationParameters& calibration, const sl::Resolution& resolution,
                                sensor_msgs::CameraInfoPtr const& leftInfoMsg, sensor_msgs::CameraInfoPtr const& rightInfoMsg) {
        leftInfoMsg->width = resolution.width;
        leftInfoMsg->height = resolution.height;
        rightInfoMsg->width = resolution.width;
        rightInfoMsg->height = resolution.height;
        leftInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        rightInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
        leftInfoMsg->D.assign(calibration.left_cam.disto, calibration.left_cam.disto + 5);
        rightInfoMsg->D.assign(calibration.right_cam.disto, calibration.right_cam.disto + 5);
        leftInfoMsg->K.fill(0.0);
        rightInfoMsg->K.fill(0.0);
        leftInfoMsg->K[0] = calibration.left_cam.fx;
        leftInfoMsg->K[2] = calibration.left_cam.cx;
        leftInfoMsg->K[4] = calibration.left_cam.fy;
        leftInfoMsg->K[5] = calibration.left_cam.cy;
        leftInfoMsg->K[8] = 1.0;
        rightInfoMsg->K[0] = calibration.right_cam.fx;
        rightInfoMsg->K[2] = calibration.right_cam.cx;
        rightInfoMsg->K[4] = calibration.right_cam.fy;
        rightInfoMsg->K[5] = calibration.right_cam.cy;
        rightInfoMsg->K[8] = 1;
        leftInfoMsg->R.fill(0.0);
        rightInfoMsg->R.fill(0.0);
        for (size_t i = 0; i < 3; ++i) leftInfoMsg->R[i * 3 + i] = 1.0;
        for (size_t i = 0; i < 3; ++i) rightInfoMsg->R[i * 3 + i] = 1.0;
        leftInfoMsg->P.fill(0.0);
        rightInfoMsg->P.fill(0.0);
        leftInfoMsg->P[0] = calibration.left_cam.fx;
        leftInfoMsg->P[2] = calibration.left_cam.cx;
        leftInfoMsg->P[5] = calibration.left_cam.fy;
        leftInfoMsg->P[6] = calibration.left_cam.cy;
        leftInfoMsg->P[10] = 1.0;
        rightInfoMsg->P[3] = -1.0 * calibration.left_cam.fx * calibration.getCameraBaseline();
        rightInfoMsg->P[0] = calibration.right_cam.fx;
        rightInfoMsg->P[2] = calibration.right_cam.cx;
        rightInfoMsg->P[5] = calibration.right_cam.fy;
        rightInfoMsg->P[6] = calibration.right_cam.cy;
        rightInfoMsg->P[10] = 1.0;
    }

} // namespace mrover
