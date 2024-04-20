#pragma once

#include <cstdint>
#include <filesystem>
#include <format>
#include <optional>
#include <stdexcept>
#include <string>

#include <XmlRpcValue.h>
#include <ros/node_handle.h>

#include "units/units.hpp"

#include "units/units.hpp"

namespace mrover {

    // Define a function template for type conversion with a default value
    template<typename T>
    auto xmlRpcValueToTypeOrDefault(XmlRpc::XmlRpcValue const& parent, std::string const& member, std::optional<T> const& defaultValue = std::nullopt) -> T {
        if (!parent.hasMember(member)) {
            if (defaultValue) return defaultValue.value();

            throw std::invalid_argument(std::format("Member not found: {}", member));
        }

        XmlRpc::XmlRpcValue const& value = parent[member];

        if constexpr (std::is_same_v<T, std::uint8_t>) {
            if (value.getType() != XmlRpc::XmlRpcValue::TypeInt) {
                throw std::invalid_argument("Expected XmlRpcValue of TypeInt for member: " + member);
            }
            return static_cast<std::uint8_t>(static_cast<int>(value));
        } else if constexpr (std::is_same_v<T, std::string> || std::is_same_v<T, std::filesystem::path>) {
            if (value.getType() != XmlRpc::XmlRpcValue::TypeString) {
                throw std::invalid_argument("Expected XmlRpcValue of TypeString for member: " + member);
            }
            return static_cast<std::string>(value);
        } else if constexpr (std::is_same_v<T, double>) {
            if (value.getType() != XmlRpc::XmlRpcValue::TypeDouble) {
                throw std::invalid_argument("Expected XmlRpcValue of TypeDouble for member: " + member);
            }
            return static_cast<double>(value);
        } else if constexpr (std::is_same_v<T, bool>) {
            if (value.getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
                throw std::invalid_argument("Expected XmlRpcValue of TypeBoolean for member: " + member);
            }
            return static_cast<bool>(value);
        } else {
            throw std::invalid_argument("Unsupported type conversion for member: " + member);
        }
    }

    template<std::size_t N>
    auto xmlRpcValueToNumberArray(XmlRpc::XmlRpcValue const& parent, std::string const& member) -> std::array<double, N> {
        if (!parent.hasMember(member)) {
            throw std::invalid_argument(std::format("Member not found: {}", member));
        }

        XmlRpc::XmlRpcValue const& value = parent[member];
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            throw std::invalid_argument(std::format("Expected XmlRpcValue of TypeArray for member: {}", member));
        }
        if (value.size() != N) {
            throw std::invalid_argument(std::format("Expected array of size {} for member: {}", N, member));
        }

        std::array<double, N> result;
        for (int i = 0; i < N; ++i) {
            if (value[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                result[i] = static_cast<double>(value[i]);
            } else if (value[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                result[i] = static_cast<int>(value[i]);
            } else {
                throw std::invalid_argument(std::format("Expected XmlRpcValue of TypeDouble or TypeInt for member: {}", member));
            }
        }
        return result;
    }

    template<IsUnit Unit>
    auto requireParamAsUnit(ros::NodeHandle const& nh, std::string const& name) -> Unit {
        assert(nh.hasParam(name));

        typename Unit::rep_t value;
        nh.getParam(name, value);
        return Unit{value};
    }

} // namespace mrover
