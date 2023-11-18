#include "logger.cuh"

namespace nvinfer1 {

    auto Logger::log(Severity severity, char const* msg) noexcept -> void {
        switch (severity) {
            case Severity::kINTERNAL_ERROR:
                ROS_FATAL_STREAM(msg);
                break;
            case Severity::kERROR:
                ROS_ERROR_STREAM(msg);
                break;
            case Severity::kWARNING:
                ROS_WARN_STREAM(msg);
                break;
            case Severity::kINFO:
                ROS_INFO_STREAM(msg);
                break;
            case Severity::kVERBOSE:
                ROS_DEBUG_STREAM(msg);
                break;
        }
    }

} // namespace nvinfer1
