#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <execution>
#include <format>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>

#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/names.h>
#include <ros/node_handle.h>
#include <ros/this_node.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <loop_profiler.hpp>
