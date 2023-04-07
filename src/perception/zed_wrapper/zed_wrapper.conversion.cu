#include "zed_wrapper.hpp"

#include <cassert>

#include <thrust/execution_policy.h>
#include <thrust/iterator/zip_iterator.h>
#include <thrust/transform.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sl/Camera.hpp>

#include <se3.hpp>

constexpr float DEG2RAD = M_PI / 180.0f;
constexpr uint64_t NS_PER_S = 1000000000;

namespace mrover {

    ros::Time slTime2Ros(sl::Timestamp t) {
        return {static_cast<uint32_t>(t.getNanoseconds() / NS_PER_S),
                static_cast<uint32_t>(t.getNanoseconds() % NS_PER_S)};
    }

    void fillPointCloudMessage(sl::Mat& xyz, sl::Mat& bgra, PointCloudGpu& pcGpu, sensor_msgs::PointCloud2Ptr const& msg) {
        assert(bgra.getWidth() >= xyz.getWidth());
        assert(bgra.getHeight() >= xyz.getHeight());
        assert(bgra.getChannels() == 4);
        assert(xyz.getChannels() == 3);
        assert(msg);

        auto bgraGpuPtr = bgra.getPtr<sl::uchar4>(sl::MEM::GPU);
        auto* xyzGpuPtr = xyz.getPtr<sl::float4>(sl::MEM::GPU);
        msg->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        msg->is_dense = false;
        msg->height = bgra.getHeight();
        msg->width = bgra.getWidth();
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
        size_t size = msg->width * msg->height;
        pcGpu.resize(size);

        Point* pcGpuPtr = pcGpu.data().get();
        auto it = thrust::make_zip_iterator(thrust::make_tuple(xyzGpuPtr, bgraGpuPtr));
        thrust::transform(thrust::device, it, it + static_cast<std::ptrdiff_t>(size),
                          pcGpuPtr,
                          [] __device__(thrust::tuple<sl::float4, sl::uchar4> const& t) -> Point {
                              sl::float4 const& xyz = thrust::get<0>(t);
                              sl::uchar4 const& bgra = thrust::get<1>(t);
                              return {xyz.x, xyz.y, xyz.z,
                                      bgra.r, bgra.g, bgra.b, bgra.a,
                                      0.0f, 0.0f, 0.0f,
                                      0.0f};
                          });

        cudaMemcpy(msg->data.data(), pcGpuPtr, size * sizeof(Point), cudaMemcpyDeviceToHost);
    }

    void fillImageMessage(sl::Mat& bgra, sensor_msgs::ImagePtr const& msg) {
        assert(bgra.getChannels() == 4);
        assert(msg);

        msg->height = bgra.getHeight();
        msg->width = bgra.getWidth();
        msg->encoding = sensor_msgs::image_encodings::BGRA8;
        msg->step = bgra.getStepBytes();
        msg->is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
        auto* bgrGpuPtr = bgra.getPtr<sl::uchar1>(sl::MEM::GPU);
        size_t size = msg->step * msg->height;
        msg->data.resize(size);
        cudaMemcpy(msg->data.data(), bgrGpuPtr, size, cudaMemcpyDeviceToHost);
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

    void fillMagMessage(sl::SensorsData::MagnetometerData& magData, sensor_msgs::MagneticField& msg) {
        R3 field{magData.magnetic_field_calibrated.x, magData.magnetic_field_calibrated.y, magData.magnetic_field_calibrated.z};
        SO3 rotation{M_PI_2, R3::UnitZ()};
        R3 rotatedField = rotation * field;

        msg.magnetic_field.x = rotatedField.x();
        msg.magnetic_field.y = rotatedField.y();
        msg.magnetic_field.z = rotatedField.z();

        msg.magnetic_field_covariance.fill(0.0f);
        msg.magnetic_field_covariance[0] = 0.039e-6;
        msg.magnetic_field_covariance[4] = 0.037e-6;
        msg.magnetic_field_covariance[8] = 0.047e-6;
    }

    void fillCameraInfoMessages(sl::CalibrationParameters& calibration, const sl::Resolution& resolution,
                                sensor_msgs::CameraInfoPtr const& leftInfoMsg, sensor_msgs::CameraInfoPtr const& rightInfoMsg) {
        assert(leftInfoMsg);
        assert(rightInfoMsg);

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
