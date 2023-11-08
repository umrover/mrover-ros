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

    class CanDevice {
    public:
        CanDevice(ros::NodeHandle const& nh, std::string from_device, std::string to_device)
            : m_nh{nh},
              m_can_publisher{m_nh.advertise<mrover::CAN>(std::format("can/{}/out", from_device), 1)},
              m_from_device{std::move(from_device)},
              m_to_device{std::move(to_device)} {
        }

        template<typename... Variants>
            requires(IsCanFdSerializable<std::variant<Variants...>>)
        std::variant<Variants...> process_message(CAN::ConstPtr const& msg) {
            // TODO.
            // You will want to call this function on the bridge node
            // void CanDevice::process_incoming_message(CAN::ConstPtr const& msg)
            // m_can_subscriber{m_nh.subscribe<CAN>(std::format("can/{}/in", can_device), 16, &CanDevice::process_incoming_message, this)}


            // This code should process the message
            // TODO - actually don't know how we should process the messages
        }

        template<typename... Variants>
            requires(IsCanFdSerializable<std::variant<Variants...>>)
        void publish_message(std::variant<Variants...> const& data) {
            // TODO - make sure everything is consistent in the bridge files
            // This is okay since "publish_data" makes a copy
            auto* address = reinterpret_cast<std::byte const*>(&data);
            // Consider a variant where one alternative is very small and the other is very large
            // We don't want to always serialize the size of the large one (e.g. if we just did sizeof the overall variant)
            // This visit ensures we get the size of the actual underlying current alternative
            std::size_t size = std::visit([](auto const& v) { return sizeof(v); }, data);
            publish_data({address, size});
        }

        void publish_moteus_frame(moteus::CanFdFrame const& frame) {
            auto* address = reinterpret_cast<std::byte const*>(frame.data);
            publish_data({address, frame.size}, frame.reply_required);
        }

    private:
        ros::NodeHandle m_nh;
        ros::Publisher m_can_publisher;
        std::string m_from_device{}, m_to_device{};

        void publish_data(std::span<std::byte const> data, bool reply_required = false) {
            mrover::CAN can_message;
            can_message.source = m_from_device;
            can_message.destination = m_to_device;
            can_message.reply_required = reply_required;
            // This is needed since ROS is old and uses std::uint8_t instead of std::byte
            std::ranges::transform(data, std::back_inserter(can_message.data), [](std::byte b) { return static_cast<std::uint8_t>(b); });
            m_can_publisher.publish(can_message);
        }
    };

} // namespace mrover
