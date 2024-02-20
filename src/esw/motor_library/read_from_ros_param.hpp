#pragma once

#include <memory.h>
#include <ros/ros.h>

namespace mrover {

    [[maybe_unused]] static auto getFloatFromRosParam(ros::NodeHandle const& nh, std::string const& paramName) -> float {
        XmlRpc::XmlRpcValue rpcValue;
        assert(nh.hasParam(paramName));
        nh.getParam(paramName, rpcValue);
        assert(rpcValue.getType() == XmlRpc::XmlRpcValue::TypeDouble);
        auto value = static_cast<float>(static_cast<double>(rpcValue));
        return value;
    }

} // namespace mrover