#pragma once

#include <cassert>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <thread>

#include <sl/Camera.hpp>
#include <thrust/device_vector.h>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <se3.hpp>
#include <point.hpp>
#include <loop_profiler.hpp>
