#pragma once

#include <XmlRpcValue.h>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>

namespace mrover {

    // Define a function template for type conversion with a default value
    template<typename T>
    T xmlRpcValueToTypeOrDefault(XmlRpc::XmlRpcValue const& parent, std::string const& member, std::optional<T> const& defaultValue = std::nullopt) {
        if (!parent.hasMember(member)) {
            if (defaultValue.has_value()) {
                return defaultValue.value();
            } else {
                throw std::invalid_argument("Member not found: " + member);
            }
        }

        XmlRpc::XmlRpcValue const& value = parent[member];

        if constexpr (std::is_same_v<T, std::uint8_t>) {
            if (value.getType() != XmlRpc::XmlRpcValue::TypeInt) {
                throw std::invalid_argument("Expected XmlRpcValue of TypeInt for member: " + member);
            }
            return static_cast<std::uint8_t>(static_cast<int>(value));
        } else if constexpr (std::is_same_v<T, std::string>) {
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


} // namespace mrover