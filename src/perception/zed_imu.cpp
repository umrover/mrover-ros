#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>

#include <sensor_msgs/Imu.h>

#include <sl/Camera.hpp>

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "zed_imu");

    sl::Camera zed;
    sl::InitParameters initParameters;
    initParameters.input.setFromSerialNumber(12046231, sl::BUS_TYPE::USB); // ZED mini
    initParameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    initParameters.coordinate_units = sl::UNIT::METER;
    initParameters.sdk_verbose = true;
    initParameters.depth_mode = sl::DEPTH_MODE::NONE;
    initParameters.camera_resolution = sl::RESOLUTION::VGA;

    if (zed.open(initParameters) != sl::ERROR_CODE::SUCCESS) {
        ROS_FATAL("Failed to open ZED camera");
        return EXIT_FAILURE;
    }

    ros::NodeHandle nh;

    ros::Publisher imuPub = nh.advertise<sensor_msgs::Imu>("/imu/data_raw", 1);

    ros::Rate rate{30};

    while (ros::ok()) {
        if (sl::SensorsData sensorData; zed.getSensorsData(sensorData, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) {
            sensor_msgs::Imu imuMsg;
            imuMsg.header.stamp = ros::Time::now();
            imuMsg.orientation.x = sensorData.imu.pose.getOrientation().x;
            imuMsg.orientation.y = sensorData.imu.pose.getOrientation().y;
            imuMsg.orientation.z = sensorData.imu.pose.getOrientation().z;
            imuMsg.orientation.w = sensorData.imu.pose.getOrientation().w;
            imuPub.publish(imuMsg);
        }
        rate.sleep();
    }

    return EXIT_SUCCESS;
}
