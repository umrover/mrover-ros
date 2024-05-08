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

#include <Eigen/Eigenvalues>

constexpr static std::size_t BUFFER_SIZE = 256, LINE_ABORT_SIZE = 4096;

static ros::Duration const STEP{0.1}, WINDOW{1};

constexpr static float CORRECTION_THRESHOLD = 1;

static std::string const BASE_LINK_FRAME = "base_link", MAP_FRAME = "map";

struct ImuData {
    S3 q = S3::Identity();
    R3 a, g, m;
    float temperature{};
    unsigned int calibration{};
};

auto operator>>(std::istream& iss, ImuData& data) -> std::istream& {
    return iss >> data.q.x() >> data.q.y() >> data.q.z() >> data.q.w() >>
           data.a.x() >> data.a.y() >> data.a.z() >>
           data.g.x() >> data.g.y() >> data.g.z() >>
           data.m.x() >> data.m.y() >> data.m.z() >>
           data.temperature >>
           data.calibration;
}

auto squared(auto const& x) { return x * x; }

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

    S3 orientationCorrection = S3::Identity();

    ImuData data;

    ros::Timer correctTimer;
    if (correct) correctTimer = nh.createTimer(WINDOW, [&](ros::TimerEvent const&) {
        Eigen::Matrix3Xd roverVelocitiesInMap; // Each column vector is a velocity

        ros::Time end = ros::Time::now(), start = end - WINDOW;
        for (ros::Time t = start; t < end; t += STEP) {
            auto roverInMapOld = SE3Conversions::fromTfTree(tfBuffer, BASE_LINK_FRAME, MAP_FRAME, t);
            auto roverInMapNew = SE3Conversions::fromTfTree(tfBuffer, BASE_LINK_FRAME, MAP_FRAME, t + STEP);
            R3 roverVelocityInMap = (roverInMapNew.translation() - roverInMapOld.translation()) / STEP.toSec();
            roverVelocitiesInMap.resize(Eigen::NoChange, roverVelocitiesInMap.cols() + 1);
            roverVelocitiesInMap.col(roverVelocitiesInMap.cols() - 1) = roverVelocityInMap;
        }

        R3 mean = roverVelocitiesInMap.rowwise().mean();
        R3 centered = roverVelocitiesInMap.colwise() - mean;
        Eigen::Matrix3d covariance = centered * centered.adjoint() / static_cast<double>(roverVelocitiesInMap.cols());

        // If the velocity has not changed too much in a local timeframe, assume we are driving straight
        if (covariance.norm() < CORRECTION_THRESHOLD) {
            R3 forward = mean.normalized();
            R3 right = R3::UnitZ().cross(forward).normalized();
            R3 up = forward.cross(right);
            SO3d::Rotation orientation;
            orientation << forward, right, up;
            orientationCorrection = orientation * data.q.inverse();
        }
    });
    (void) correctTimer;

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
            std::size_t newLineIndex = remaining.find('\n');
            if (newLineIndex == std::string::npos) break;

            std::string line = remaining.substr(0, newLineIndex);
            remaining = remaining.substr(newLineIndex + 1);

            // Means we have not seen a new line in a while, IMU firmware may be giving wrong data
            if (remaining.size() > LINE_ABORT_SIZE) throw std::runtime_error("Too much data");

            if (std::istringstream iss{line}; !(iss >> data)) {
                ROS_ERROR_STREAM("Failed to parse IMU data: " << line);
                continue;
            }

            S3 correctedOrientation = data.q * orientationCorrection;

            mrover::ImuAndMag msg{};
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = frame;
            msg.imu.orientation.x = correctedOrientation.x();
            msg.imu.orientation.y = correctedOrientation.y();
            msg.imu.orientation.z = correctedOrientation.z();
            msg.imu.orientation.w = correctedOrientation.w();
            msg.imu.angular_velocity.x = data.g.x();
            msg.imu.angular_velocity.y = data.g.y();
            msg.imu.angular_velocity.z = data.g.z();
            msg.imu.linear_acceleration.x = data.a.x();
            msg.imu.linear_acceleration.y = data.a.y();
            msg.imu.linear_acceleration.z = data.a.z();
            msg.mag.header.stamp = ros::Time::now();
            msg.mag.header.frame_id = frame;
            msg.mag.magnetic_field.x = data.m.x();
            msg.mag.magnetic_field.y = data.m.y();
            msg.mag.magnetic_field.z = data.m.z();
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