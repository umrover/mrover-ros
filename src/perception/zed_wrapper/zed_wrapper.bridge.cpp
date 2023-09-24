/*
 * Most functions in here are copied from: https://github.com/stereolabs/zed-ros-wrapper/blob/master/zed_nodelets/src/zed_nodelet/src/zed_wrapper_nodelet.cpp
 */

#include "zed_wrapper.hpp"

namespace mrover {

    constexpr float DEG2RAD = M_PI / 180.0f;
    constexpr uint64_t NS_PER_S = 1000000000;

    ros::Time slTime2Ros(sl::Timestamp t) {
        return {static_cast<uint32_t>(t.getNanoseconds() / NS_PER_S),
                static_cast<uint32_t>(t.getNanoseconds() % NS_PER_S)};
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
        checkCudaError(cudaMemcpy(msg->data.data(), bgrGpuPtr, size, cudaMemcpyDeviceToHost));
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
        msg.magnetic_field.x = magData.magnetic_field_calibrated.x;
        msg.magnetic_field.y = magData.magnetic_field_calibrated.y;
        msg.magnetic_field.z = magData.magnetic_field_calibrated.z;

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
        leftInfoMsg->D.resize(5);
        rightInfoMsg->D.resize(5);
        leftInfoMsg->D[0] = calibration.left_cam.disto[0];
        leftInfoMsg->D[1] = calibration.left_cam.disto[1];
        leftInfoMsg->D[2] = calibration.left_cam.disto[4];
        leftInfoMsg->D[3] = calibration.left_cam.disto[2];
        leftInfoMsg->D[4] = calibration.left_cam.disto[3];
        rightInfoMsg->D[0] = calibration.right_cam.disto[0];
        rightInfoMsg->D[1] = calibration.right_cam.disto[1];
        rightInfoMsg->D[2] = calibration.right_cam.disto[4];
        rightInfoMsg->D[3] = calibration.right_cam.disto[2];
        rightInfoMsg->D[4] = calibration.right_cam.disto[3];
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
        for (size_t i = 0; i < 3; ++i) {
            leftInfoMsg->R[i * 3 + i] = 1.0;
            rightInfoMsg->R[i * 3 + i] = 1.0;
        }
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
