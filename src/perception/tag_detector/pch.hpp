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
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/SetBool.h>

#include <mrover/DetectorParamsConfig.h>
#include <loop_profiler.hpp>
#include <lie.hpp>