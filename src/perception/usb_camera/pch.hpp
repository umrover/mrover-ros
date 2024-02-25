#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <execution>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>

#include <opencv2/aruco.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <dynamic_reconfigure/server.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/names.h>
#include <ros/this_node.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/SetBool.h>

#include <loop_profiler.hpp>
