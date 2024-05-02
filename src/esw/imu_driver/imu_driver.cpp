#include <cstdlib>
#include <ros/init.h>
#include <ros/node_handle.h>

#include <fcntl.h>
#include <ros/publisher.h>
#include <termios.h>

#include <sstream>
#include <array>

#include <mrover/ImuAndMag.h>

constexpr std::size_t BUFFER_SIZE = 1024;

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

    ros::Publisher pub = nh.advertise<mrover::ImuAndMag>("/imu/data", 1);

    int fd = open("/dev/imu", O_RDWR);
    if (fd < 0) {
        ROS_ERROR("Failed to open IMU device");
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
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        return EXIT_FAILURE;
    }

    std::string remaining;

    while (ros::ok()) {
        std::array<char, BUFFER_SIZE> buffer{};
        ssize_t actually_read = read(fd, buffer.data(), buffer.size());
        if (actually_read == 0) throw std::runtime_error("EOF");

        remaining.append(buffer.data(), actually_read);

        while (true) {
            auto pos = remaining.find('\n');
            if (pos == std::string::npos) break;

            std::string line = remaining.substr(0, pos);
            remaining = remaining.substr(pos + 1);

            if (remaining.size() > 2048) throw std::runtime_error("Too much data");

            ImuData data{};

            std::istringstream iss{line};
            if (!(iss >> data.qx >> data.qy >> data.qz >> data.qw
                      >> data.ax >> data.ay >> data.az
                      >> data.gx >> data.gy >> data.gz
                      >> data.mx >> data.my >> data.mz
                      >> data.temperature >> data.calibration)) {
                ROS_ERROR_STREAM("Failed to parse IMU data: " << line);
                continue;
            }

            mrover::ImuAndMag msg{};
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";
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
            msg.mag.magnetic_field.x = data.mx;
            msg.mag.magnetic_field.y = data.my;
            msg.mag.magnetic_field.z = data.mz;
            pub.publish(msg);
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