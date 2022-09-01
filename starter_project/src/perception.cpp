#include "perception.hpp"

// C++ Standard Library Headers, std namespace
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

// OpenCV Headers, cv namespace
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>

// ROS Headers, ros namespace
#include <image_transport/image_transport.h>
#include <ros/init.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "perception_starter_project");
    ros::NodeHandle nodeHandle("starter_project"); // Set namespace (See: http://wiki.ros.org/Names)

    image_transport::ImageTransport imageTransport(nodeHandle);
    imageTransport.subscribe("camera/color/image_raw", 1, &imageCallback);

    ros::spin();
}

void imageCallback(const sensor_msgs::ImageConstPtr& image) {
}
