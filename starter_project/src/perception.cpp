#include "perception.hpp"

// C++ Standard Library Headers, std namespace
#include <string>
#include <memory>
#include <optional>
#include <unordered_map>

// OpenCV Headers, cv namespace
#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>

// ROS Headers, ros namespace
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "perception_starter_project");



    ros::spin();
}