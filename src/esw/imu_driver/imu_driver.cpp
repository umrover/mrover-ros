#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <fcntl.h>
#include <termios.h>

#include <array>
#include <cstdlib>
#include <sstream>

#include <sensor_msgs/Temperature.h>

#include <mrover/CalibrationStatus.h>
#include <mrover/ImuAndMag.h>

constexpr std::size_t BUFFER_SIZE = 256;
constexpr std::size_t LINE_ABORT_SIZE = 4096;

struct ImuData {
    float qx, qy, qz, qw;
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float temperature;
    unsigned int calibration;
};

// TODO(quintin): Why doesn't this work?
// auto operator>>(std::istringstream& iss, ImuData& data) -> std::istringstream& {
//     return iss >> data.qx >> data.qy >> data.qz >> data.qw
//                >> data.ax >> data.ay >> data.az
//                >> data.gx >> data.gy >> data.gz
//                >> data.mx >> data.my >> data.mz
//                >> data.temperature >> data.calibration;
// }

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "imu_driver");
    ros::NodeHandle nh;

    ros::Publisher dataPublisher = nh.advertise<mrover::ImuAndMag>("/imu/data", 1);
    ros::Publisher calibrationPublisher = nh.advertise<mrover::CalibrationStatus>("/imu/calibration", 1);
    ros::Publisher temperaturePublisher = nh.advertise<sensor_msgs::Temperature>("/imu/temperature", 1);

    auto port = nh.param<std::string>("/imu_driver/port", "/dev/imu");
    auto frame = nh.param<std::string>("/imu_driver/frame_id", "imu_link");

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

        while (true) {
            auto pos = remaining.find('\n');
            if (pos == std::string::npos) break;

            std::string line = remaining.substr(0, pos);
            remaining = remaining.substr(pos + 1);

            // Means we have not seen a new line in a while, IMU firmware may be giving wrong data
            if (remaining.size() > LINE_ABORT_SIZE) throw std::runtime_error("Too much data");

            ImuData data{};

            if (std::istringstream iss{line}; !(iss >> data.qx >> data.qy >> data.qz >> data.qw >> data.ax >> data.ay >> data.az >> data.gx >> data.gy >> data.gz >> data.mx >> data.my >> data.mz >> data.temperature >> data.calibration)) {
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