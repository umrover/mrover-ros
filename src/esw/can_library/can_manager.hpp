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

#define CANFD_FDF 0x04
#include <moteus/moteus.h>

namespace mrover {

    using namespace mjbots;

    // Attribute needed to pack the struct into 32 bits
    struct FDCANMessageID {
        [[maybe_unused]] std::uint8_t _ignore : 3;
        std::uint16_t padding : 13;      // padding
        std::uint8_t source_id : 8;      // source id
        std::uint8_t destination_id : 8; // destination id
    } __attribute__((__packed__));

    template<typename T>
    concept IsSerializable = std::is_trivially_copyable_v<T>;

    template<typename T>
    concept IsFDCANSerializable = IsSerializable<T> && sizeof(T) <= 64;

    class CANManager {
    public:
        CANManager(ros::NodeHandle& nh, const std::string& name) {
            m_can_publisher = nh.advertise<mrover::CAN>("can_requests", 1);
            std::string can_bus_name = "can/" + name + "/bus";
            assert(nh.hasParam(can_bus_name));
            int bus;
            nh.getParam(can_bus_name, bus);
            m_bus = static_cast<int>(bus);
            std::string can_id_name = "can/" + name + "/id";
            assert(nh.hasParam(can_id_name));
            int id;
            nh.getParam(can_id_name, id);
            m_id = static_cast<int>(id);

            XmlRpc::XmlRpcValue can_messages;
            assert(nh.hasParam("can/messages"));
            nh.getParam("can/messages", can_messages);
            assert(can_messages.getType() == XmlRpc::XmlRpcValue::TypeStruct);
            for (auto [messageName, messageId]: can_messages) {
                if (messageId.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                    auto messageIdInt = static_cast<int>(messageId);
                    m_message_name_to_id[messageName] = messageIdInt;
                    m_id_to_message_name[messageIdInt] = messageName;
                }
            }
        }

        template<IsFDCANSerializable T>
        void send_data(std::string const& messageName, T& data) {
            // This is okay since "send_raw_data" makes a copy
            auto* address = std::bit_cast<std::byte const*>(std::addressof(data));
            send_raw_data(messageName, {address, sizeof(data)});
        }

        void send_raw_data(std::string const& messageName, std::span<std::byte const> data) {
            if (!m_message_name_to_id.contains(messageName)) {
                throw std::invalid_argument(std::format("message_name {} is not valid.", messageName));
            }

            mrover::CAN can_message;
            can_message.bus = m_bus;
            can_message.message_id = std::bit_cast<uint16_t>(FDCANMessageID{
                    .message_num = m_message_name_to_id[messageName],
                    .device_id = m_id,
            });
            can_message.data.resize(data.size());
            std::memcpy(can_message.data.data(), data.data(), data.size());

            m_can_publisher.publish(can_message);
        }

        void send_moteus_data(moteus::CanFdFrame const& frame) {
            mrover::CAN can_message;
            can_message.bus = frame.bus;
            struct MoteusId {
                std::int8_t destination;
                std::int8_t source : 7;
                bool reply_required : 1;
            } __attribute__((__packed__));
            can_message.message_id = std::bit_cast<std::uint16_t>(MoteusId{
                    .destination = frame.destination,
                    .source = frame.source,
                    .reply_required = frame.reply_required,
            });
            can_message.data.assign(frame.data, frame.data + frame.size);

            m_can_publisher.publish(can_message);
        }

        uint8_t get_id() const {
            return m_id;
        }

        uint8_t get_bus() const {
            return m_bus;
        }

        std::string get_message(int messageId) {
            return m_id_to_message_name[messageId];
        }

    private:
        ros::Publisher m_can_publisher;
        uint8_t m_bus{}; // Who sets/determines the bus field?
        uint8_t m_id{};  // Who sets this/how does it relate to the devices
        // ^^Change the wording of m_id to imply this is the destination id
        // Guthrie assigns an ID and we then use the moteus gui to configure the
        // id of the moteus to the ID Guthrie assigns.
        std::unordered_map<std::string, uint8_t> m_message_name_to_id;
        std::unordered_map<uint8_t, std::string> m_id_to_message_name;
    };

} // namespace mrover
