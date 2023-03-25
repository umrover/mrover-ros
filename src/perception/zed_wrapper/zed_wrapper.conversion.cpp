#include "zed_wrapper.hpp"

#include <algorithm>
#include <execution>
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

    void fillImuMessage(sl::SensorsData::IMUData& imuData, sensor_msgs::Imu& msg, size_t tick) {
        msg.header.stamp = ros::Time::now();
        msg.header.seq = tick;
        msg.header.frame_id = "zed2i_imu_frame";
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

} // namespace mrover
