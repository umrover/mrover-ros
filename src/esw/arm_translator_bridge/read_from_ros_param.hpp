#pragma once

#include <memory.h>
#include <ros/ros.h>

namespace mrover {

    static float getFloatFromRosParam(ros::NodeHandle const& nh, std::string const& param_name) {
        XmlRpc::XmlRpcValue rpcValue;
        assert(nh.hasParam(param_name));
        nh.getParam(param_name, rpcValue);
        assert(rpcValue.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        auto value = (float) static_cast<double>(rpcValue);
        return value;
    }

} // namespace mrover