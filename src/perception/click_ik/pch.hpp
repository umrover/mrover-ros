#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <execution>
#include <limits>
#include <numeric>
#include <optional>
#include <string>
#include <type_traits>
#include <unordered_map>

#include <dynamic_reconfigure/server.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_srvs/SetBool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <loop_profiler.hpp>
#include <point.hpp>
#include <lie.hpp>

#include <mrover/ClickIkAction.h>
