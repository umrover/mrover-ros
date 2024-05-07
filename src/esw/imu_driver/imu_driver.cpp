#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <lie.hpp>

#include <sensor_msgs/Temperature.h>

#include <fcntl.h>
#include <termios.h>

#include <array>
#include <cstdlib>
#include <numeric>
#include <sstream>

#include <mrover/CalibrationStatus.h>
#include <mrover/ImuAndMag.h>

#include <boost/circular_buffer.hpp>

constexpr static std::size_t BUFFER_SIZE = 256;
constexpr static std::size_t LINE_ABORT_SIZE = 4096;

struct ImuData {
    float qx, qy, qz, qw;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float temperature;
    unsigned int calibration;
};

auto operator>>(std::istream& iss, ImuData& data) -> std::istream& {
    return iss >> data.qx >> data.qy >> data.qz >> data.qw >> data.ax >> data.ay >> data.az >> data.gx >> data.gy >> data.gz >> data.mx >> data.my >> data.mz >> data.temperature >> data.calibration;
}

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "imu_driver");
    ros::NodeHandle nh;

    ros::Publisher dataPublisher = nh.advertise<mrover::ImuAndMag>("/imu/data", 1);
    ros::Publisher calibrationPublisher = nh.advertise<mrover::CalibrationStatus>("/imu/calibration", 1);
    ros::Publisher temperaturePublisher = nh.advertise<sensor_msgs::Temperature>("/imu/temperature", 1);

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};

    auto port = nh.param<std::string>("/imu_driver/port", "/dev/imu");
    auto frame = nh.param<std::string>("/imu_driver/frame_id", "imu_link");
    auto correct = nh.param<bool>("/imu_driver/correct", true);

    float correctionOffset = 0;

    constexpr ros::Duration step{0.1};
    constexpr ros::Duration window{1};
    ros::Timer correctTimer = nh.createTimer(window, [&](ros::TimerEvent const&) {
        std::vector<float> angles;
        ros::Time end = ros::Time::now(), start = end - window;
        for (ros::Time t = start; t < end; t += step) {
            auto t1 = SE3Conversions::fromTfTree(tfBuffer, "base_link", "imu_link", t);
            auto t2 = SE3Conversions::fromTfTree(tfBuffer, "base_link", "imu_link", t + step);
            auto v = (t2.translation() - t1.translation()) / step.toSec();
            angles.push_back(std::atan2(v.y(), v.x()));
        }

        float mean = std::accumulate(angles.begin(), angles.end(), 0.0f) / static_cast<float>(angles.size());
        float variance = std::accumulate(angles.begin(), angles.end(), 0.0f, [mean](float acc, float angle) {
                             return acc + (angle - mean) * (angle - mean);
                         }) /
                         static_cast<float>(angles.size());
        if (float stddev = std::sqrt(variance); stddev < 1) {
            correctionOffset = mean;
        }
    });

    int fd = open(port.c_str(), O_RDWR);
    if (fd < 0) {
        ROS_ERROR("Failed to open IMU device! Check that udev is set up properly and that the device is connected.");
        return EXIT_FAILURE;
    }

    termios tty{};
    if (tcgetattr(fd, &tty) < 0) {
        ROS_ERROR("Failed to get IMU device attributes");
        return EXIT_FAILURE;
    }

    // Note(quintin): I printed the "tty" values after configuring the port with minicom:
    // tty.c_cflag = 3261;
    // tty.c_iflag = 1280;
    // tty.c_oflag = 5;

    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR;

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        ROS_ERROR_STREAM("Error " << errno << " from tcsetattr: " << strerror(errno));
        return EXIT_FAILURE;
    }

    std::string remaining;

    while (ros::ok()) {
        // Idea: Read in a chunk of data and then extract as many lines as possible
        //       Reading byte-by-byte is inefficient due to the number of syscalls

        std::array<char, BUFFER_SIZE> buffer{};
        ssize_t actually_read = read(fd, buffer.data(), buffer.size());
        if (actually_read == 0) throw std::runtime_error("EOF");

        remaining.append(buffer.data(), actually_read);

        while (ros::ok()) {
            auto pos = remaining.find('\n');
            if (pos == std::string::npos) break;

            std::string line = remaining.substr(0, pos);
            remaining = remaining.substr(pos + 1);

            // Means we have not seen a new line in a while, IMU firmware may be giving wrong data
            if (remaining.size() > LINE_ABORT_SIZE) throw std::runtime_error("Too much data");

            ImuData data{};

            if (std::istringstream iss{line}; !(iss >> data)) {
                ROS_ERROR_STREAM("Failed to parse IMU data: " << line);
                continue;
            }

            mrover::ImuAndMag msg{};
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = frame;
            msg.imu.orientation.x = data.qx;
            msg.imu.orientation.y = data.qy;
            msg.imu.orientation.z = data.qz;
            msg.imu.orientation.w = data.qw;
            msg.imu.angular_velocity.x = data.gx;
            msg.imu.angular_velocity.y = data.gy;
            msg.imu.angular_velocity.z = data.gz;
            msg.imu.linear_acceleration.x = data.ax;
            msg.imu.linear_acceleration.y = data.ay;
            msg.imu.linear_acceleration.z = data.az;
            msg.mag.header.stamp = ros::Time::now();
            msg.mag.header.frame_id = frame;
            msg.mag.magnetic_field.x = data.mx;
            msg.mag.magnetic_field.y = data.my;
            msg.mag.magnetic_field.z = data.mz;
            dataPublisher.publish(msg);

            mrover::CalibrationStatus calibration{};
            calibration.header.stamp = ros::Time::now();
            calibration.header.frame_id = frame;
            calibration.system_calibration = data.calibration;
            calibrationPublisher.publish(calibration);

            sensor_msgs::Temperature temperature{};
            temperature.header.stamp = ros::Time::now();
            temperature.header.frame_id = frame;
            temperature.temperature = data.temperature;
            temperaturePublisher.publish(temperature);
        }

        ros::spinOnce();
    }

    // If this is not present, the next serial connection may block for some reason
    // Copied this from minicom source code: https://github.com/Distrotech/minicom
    tcgetattr(fd, &tty);
    tty.c_cflag &= ~HUPCL;
    tcsetattr(fd, TCSANOW, &tty);

    close(fd);

    return EXIT_SUCCESS;
}