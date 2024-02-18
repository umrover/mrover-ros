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

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <dynamic_reconfigure/server.h>
#include <mrover/DetectorParamsConfig.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/publisher.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/SetBool.h>

#include <loop_profiler.hpp>
#include <lie.hpp>

#include <mrover/LongRangeTag.h>
#include <mrover/LongRangeTags.h>
