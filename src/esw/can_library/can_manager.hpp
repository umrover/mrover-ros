#pragma once

#include <algorithm>
#include <cstdint>
#include <format>
#include <span>
#include <string>
#include <unordered_map>

#include <XmlRpcValue.h>
#include <ros/ros.h>

#include <bimap.hpp>
#include <mrover/CAN.h>

#define CANFD_FDF 0x04
#include <moteus/moteus.h>

namespace mrover {

    using namespace mjbots;

    struct FdcanMessageId {
        std::uint8_t destination;
        std::uint8_t source : 7;
        bool reply_required : 1;
    };
    static_assert(sizeof(FdcanMessageId) == 2);

    template<typename T>
    concept IsSerializable = std::is_trivially_copyable_v<T>;

    template<typename T>
    concept IsFDCANSerializable = IsSerializable<T> && sizeof(T) <= 64;

    struct FdcanDeviceAddress {
        std::uint8_t bus;
        std::uint8_t id;

        bool operator<=>(FdcanDeviceAddress const& other) const = default;
    };

    class CANManager {
    public:
        CANManager(ros::NodeHandle const& nh, std::string source_name)
            : m_nh{nh},
              m_source_name{std::move(source_name)},
              m_can_publisher{m_nh.advertise<mrover::CAN>("can_requests", 1)} {

            XmlRpc::XmlRpcValue canDevices;
            m_nh.getParam("can/devices", canDevices);
            assert(m_nh.hasParam("can/devices"));
            assert(canDevices.getType() == XmlRpc::XmlRpcValue::TypeArray);
            int size = canDevices.size();
            for (int i = 0; i < size; ++i) {
                assert(canDevices[i].hasMember("name") &&
                       canDevices[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
                std::string name = static_cast<std::string>(canDevices[i]["name"]);

                assert(canDevices[i].hasMember("bus") &&
                       canDevices[i]["bus"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                auto bus = static_cast<std::uint8_t>(static_cast<int>(canDevices[i]["bus"]));

                assert(canDevices[i].hasMember("id") &&
                       canDevices[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                auto id = static_cast<std::uint8_t>(static_cast<int>(canDevices[i]["id"]));

                m_devices.emplace(name, FdcanDeviceAddress{bus, id});
            }
        }

        template<typename... Variants>
            requires(IsFDCANSerializable<std::variant<Variants...>>)
        void send_message(std::string const& to_device, std::variant<Variants...> const& data) {
            // TODO - make sure everything is consistent in the bridge files
            // This is okay since "send_raw_data" makes a copy
            auto* address = reinterpret_cast<std::byte const*>(std::addressof(data));
            std::size_t size = sizeof(data);
            send_raw_data(to_device, {address, size});
        }

        void send_raw_data(std::string const& to_device, std::span<std::byte const> data) {
            if (!m_devices.contains(to_device)) {
                throw std::invalid_argument(std::format("CAN destination device {} does not exist", to_device));
            }

            FdcanDeviceAddress const& source = m_devices.at(m_source_name);
            FdcanDeviceAddress const& destination = m_devices.at(to_device);
            assert(source.bus == destination.bus);

            mrover::CAN can_message;
            can_message.bus = destination.bus;
            can_message.message_id = std::bit_cast<std::uint16_t>(FdcanMessageId{
                    .destination = destination.id,
                    .source = source.id,
            });
            std::ranges::transform(data, std::back_inserter(can_message.data), [](std::byte b) { return static_cast<std::uint8_t>(b); });

            m_can_publisher.publish(can_message);
        }

        void send_moteus_frame(moteus::CanFdFrame const& frame) {
            mrover::CAN can_message;
            can_message.bus = frame.bus;
            can_message.message_id = std::bit_cast<std::uint16_t>(FdcanMessageId{
                    .destination = static_cast<std::uint8_t>(frame.destination),
                    .source = static_cast<std::uint8_t>(frame.source),
                    .reply_required = frame.reply_required,
            });
            can_message.data.assign(frame.data, frame.data + frame.size);

            m_can_publisher.publish(can_message);
        }

    private:
        ros::NodeHandle m_nh;
        ros::Publisher m_can_publisher;
        std::string m_source_name{};

        bimap<std::string, FdcanDeviceAddress,
              std::hash<std::string>, decltype([](FdcanDeviceAddress const& location) {
                  return std::hash<std::uint8_t>{}(location.bus) ^ std::hash<std::uint8_t>{}(location.id);
              })>
                m_devices;
    };

} // namespace mrover
