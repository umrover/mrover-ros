#pragma once

#include <memory.h>
#include <ros/ros.h>

namespace mrover {

    static std::unique_ptr<float> get_unique_float_from_ros_param(ros::NodeHandle const& nh, std::string const& param_name) {
        XmlRpc::XmlRpcValue rpcValue;
        assert(nh.hasParam(param_name));
        nh.getParam(param_name, rpcValue);
        assert(rpcValue.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        auto value = (float) static_cast<double>(rpcValue);
        return std::make_unique<float>(value);
    }

} // namespace mrover