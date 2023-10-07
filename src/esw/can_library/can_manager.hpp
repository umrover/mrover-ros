#pragma once

#include <algorithm>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <unordered_map>

#include <XmlRpcValue.h>
#include <ros/ros.h>

#include <mrover/CAN.h>

// Attribute needed to pack the struct into 16 bits
struct __attribute__((__packed__)) MessageID {
    [[maybe_unused]] uint8_t _ignore : 5; // 16 bits - (6 + 5 meaningful bits) = 5 ignored bits
    uint8_t message_num : 6;              // 6 bits for message number
    uint8_t device_id : 5;                // 5 bits for device ID
};

template<typename T>
concept TriviallyCopyable = std::is_trivially_copyable_v<T>;

class CANManager {
public:
    CANManager(ros::NodeHandle& n, const std::string& name) {
        m_can_publisher = n.advertise<mrover::CAN>("can_requests", 1);
        std::string can_bus_name = "can/" + name + "/bus";
        assert(n.getParam(can_bus_name, m_bus));
        std::string can_id_name = "can/" + name + "/id";
        assert(n.getParam(can_id_name, m_id));

        assert(n.hasParam("can/messages"));
        XmlRpc::XmlRpcValue can_messages;
        n.getParam("can/messages", can_messages);
        assert(can_messages.getType() == XmlRpc::XmlRpcValue::TypeStruct);
        for (auto [messageName, messageId]: can_messages) {
            if (messageId.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                m_message_name_to_id[messageName] = static_cast<int>(messageId);
            }
        }
    }

    template<TriviallyCopyable T>
    void send_data(std::string const& messageName, T& data) {
        // This is okay since "send_raw_data" makes a copy
        auto* address = reinterpret_cast<std::byte*>(&data);
        send_raw_data(messageName, {address, sizeof(T)});
    }

    void send_raw_data(std::string const& messageName, std::span<std::byte> data) {
        if (!m_message_name_to_id.contains(messageName)) {
            throw std::invalid_argument(std::format("message_name {} is not valid.", messageName));
        }

        mrover::CAN can_message;
        can_message.bus = m_bus;
        can_message.message_id = std::bit_cast<uint16_t>(MessageID{
                .message_num = m_message_name_to_id[messageName],
                .device_id = m_id,
        });
        // This is ugly but needed since ROS has no std::byte message type
        can_message.data.resize(data.size());
        std::memcpy(can_message.data.data(), data.data(), data.size());

        m_can_publisher.publish(can_message);
    }

    uint8_t get_id() const {
        return m_id;
    }

    uint8_t get_bus() const {
        return m_bus;
    }

private:
    ros::Publisher m_can_publisher;
    uint8_t m_bus{};
    uint8_t m_id{};
    std::unordered_map<std::string, uint8_t> m_message_name_to_id;
};