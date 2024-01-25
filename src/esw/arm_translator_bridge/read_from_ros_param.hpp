#pragma once

#include <memory.h>
#include <ros/ros.h>

namespace mrover {

    static float getFloatFromRosParam(ros::NodeHandle const& nh, std::string const& paramName) {
        XmlRpc::XmlRpcValue rpcValue;
        assert(nh.hasParam(paramName));
        nh.getParam(paramName, rpcValue);
        assert(rpcValue.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        auto value = (float) static_cast<double>(rpcValue);
        return value;
    }

} // namespace mrover