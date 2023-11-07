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

    template<typename T>
    concept IsSerializable = std::is_trivially_copyable_v<T>;

    template<typename T>
    concept IsCanFdSerializable = IsSerializable<T> && sizeof(T) <= 64;

    struct CanFdMessageId {
        std::uint8_t destination{};
        std::uint8_t source : 7 {};
        bool reply_required : 1 {};
    };
    static_assert(sizeof(CanFdMessageId) == 2);

    struct CanFdDeviceAddress {
        std::uint8_t bus{}, id{};

        bool operator<=>(CanFdDeviceAddress const& other) const = default;
    };

    class CanFdBus {
    public:
        CanFdBus(ros::NodeHandle const& nh, std::string for_device)
            : m_nh{nh},
              m_for_device{std::move(for_device)},
              m_can_publisher{m_nh.advertise<mrover::CAN>("can_requests", 1)} {

            XmlRpc::XmlRpcValue canDevices;
            m_nh.getParam("can/devices", canDevices);
            assert(m_nh.hasParam("can/devices"));
            assert(canDevices.getType() == XmlRpc::XmlRpcValue::TypeArray);

            int size = canDevices.size();
            for (int i = 0; i < size; ++i) {
                assert(canDevices[i].hasMember("name"));
                assert(canDevices[i]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
                std::string name = static_cast<std::string>(canDevices[i]["name"]);

                assert(canDevices[i].hasMember("bus"));
                assert(canDevices[i]["bus"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                auto bus = static_cast<std::uint8_t>(static_cast<int>(canDevices[i]["bus"]));

                assert(canDevices[i].hasMember("id"));
                assert(canDevices[i]["id"].getType() == XmlRpc::XmlRpcValue::TypeInt);
                auto id = static_cast<std::uint8_t>(static_cast<int>(canDevices[i]["id"]));

                m_devices.emplace(name, CanFdDeviceAddress{bus, id});
            }
        }

        template<typename... Variants>
            requires(IsCanFdSerializable<std::variant<Variants...>>)
        void publish_message(std::string const& to_device, std::variant<Variants...> const& data) {
            // TODO - make sure everything is consistent in the bridge files
            // This is okay since "send_raw_data" makes a copy
            auto* address = reinterpret_cast<std::byte const*>(std::addressof(data));
            publish_data(to_device, {address, sizeof(data)});
        }

        void publish_moteus_frame(std::string const& to_device, moteus::CanFdFrame const& frame) {
            auto* address = reinterpret_cast<std::byte const*>(frame.data);
            publish_data(to_device, {address, frame.size}, frame.reply_required);
        }

        void publish_data(std::string const& to_device, std::span<std::byte const> data, bool reply_required = false) {
            if (!m_devices.contains(to_device)) {
                throw std::invalid_argument(std::format("CAN destination device {} does not exist", to_device));
            }

            CanFdDeviceAddress const& source = m_devices.at(m_for_device);
            CanFdDeviceAddress const& destination = m_devices.at(to_device);
            assert(source.bus == destination.bus);

            mrover::CAN can_message;
            can_message.bus = destination.bus;
            can_message.message_id = std::bit_cast<std::uint16_t>(CanFdMessageId{
                    .destination = destination.id,
                    .source = source.id,
                    .reply_required = reply_required,
            });
            std::ranges::transform(data, std::back_inserter(can_message.data), [](std::byte b) { return static_cast<std::uint8_t>(b); });

            m_can_publisher.publish(can_message);
        }

    private:
        ros::NodeHandle m_nh;
        ros::Publisher m_can_publisher;
        std::string m_for_device{};

        bimap<std::string, CanFdDeviceAddress,
              std::hash<std::string>, decltype([](CanFdDeviceAddress const& location) {
                  return std::hash<std::uint8_t>{}(location.bus) ^ std::hash<std::uint8_t>{}(location.id);
              })>
                m_devices;
    };

} // namespace mrover
