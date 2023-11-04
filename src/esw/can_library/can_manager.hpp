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
        CANManager(ros::NodeHandle& nh, const std::string& source_name) {
            m_source_name = source_name;
            m_can_publisher = nh.advertise<mrover::CAN>("can_requests", 1);

            XmlRpc::XmlRpcValue canDevices;
            nh.getParam("can/devices", canDevices);
            assert(nh.hasParam("can/devices"));
            assert(can_messages.getType() == XmlRpc::XmlRpcValue::TypeStruct);
            int32_t size = canDevices.size();
            for (int32_t i = 0; i < size; ++i) {
                assert(canDevices[i].hasMember("name") &&
                       canDevices[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
                std::string name = static_cast<std::string>(canDevices[i]["name"]);

                assert(canDevices[i].hasMember("id") &&
                       canDevices[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                auto id = (uint8_t) static_cast<int>(canDevices[i]["id"]);
                m_name_to_id[name] = id;
                m_id_to_name[id] = name;

                assert(canDevices[i].hasMember("bus") &&
                       canDevices[i]["bus"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                auto bus = (uint8_t) static_cast<int>(canDevices[i]["bus"]);
                m_name_to_bus[name] = bus;
                m_bus_to_name[bus] = name;
            }
        }

        template<IsFDCANSerializable T>
        void send_data(std::string const& destinationName, T& data) {
            // TODO - make sure everything is consistent in the bridge files
            // This is okay since "send_raw_data" makes a copy
            auto* address = std::bit_cast<std::byte const*>(std::addressof(data));
            send_raw_data(destinationName, {address, sizeof(data)});
        }

        void send_raw_data(std::string const& destinationName, std::span<std::byte const> data) {
            if (!m_name_to_bus.contains(destinationName) || !m_name_to_id.contains(destinationName)) {
                throw std::invalid_argument(std::format("destinationName {} is not valid.", destinationName));
            }


            mrover::CAN can_message;
            can_message.bus = m_name_to_bus[destinationName];
            can_message.message_id = std::bit_cast<uint16_t>(FDCANMessageID{
                    .padding = 0,
                    .source_id = m_name_to_id[m_source_name],
                    .destination_id = m_name_to_id[destinationName],
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

    private:
        ros::Publisher m_can_publisher;
        std::string m_source_name{};
        std::unordered_map<std::string, uint8_t> m_name_to_id;
        std::unordered_map<uint8_t, std::string> m_id_to_name;
        std::unordered_map<std::string, uint8_t> m_name_to_bus;
        std::unordered_map<uint8_t, std::string> m_bus_to_name;
    };

} // namespace mrover
