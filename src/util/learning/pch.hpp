#pragma once

// STL
#include <array>
#include <exception>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/SetBool.h>
#include <tf/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <loop_profiler.hpp>

#include "inference_wrapper.hpp"
