#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include <mrover/CAN.h>

struct MessageID {
    uint8_t ignore: 5; // 16 bits - (6 + 5 meaningful bits) = 5 ignored bits
    uint8_t message_num : 6; // 6 bits for message number
    uint8_t device_id : 5;   // 5 bits for device ID
};

union FloatMessage{
    float floatValue;
    uint8_t bytes[sizeof(float)];
};

union BoolMessage{
    bool boolValue;
    uint8_t bytes[sizeof(bool)];
};

std::vector<uint8_t> createFloatMessage(float floatValue);
std::vector<uint8_t> createBoolMessage(bool boolValue);

class CANManager {
public:
    CANManager(ros::NodeHandle& n, const std::string& name) {
        CANPublisher = n.advertise<mrover::CAN>("can_requests", 1);
        std::string can_bus_name = "can/" + name + "/bus";
        assert(n.getParam(can_bus_name, bus));
        std::string can_id_name = "can/" + name + "/id";
        assert(n.getParam(can_id_name, id));

        if (n.hasParam("can/messages")) {
            XmlRpc::XmlRpcValue can_messages;
            n.getParam("can/messages", can_messages);

            if (can_messages.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = can_messages.begin(); it != can_messages.end(); ++it) {
                    if (it->second.getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        message_num_by_name[it->first] = static_cast<int>(it->second);
                    }
                }
            } else {
                ROS_ERROR("YAML parameter is not in the expected format (TypeStruct).");
            }
        } else {
            ROS_ERROR("Failed to retrieve the YAML parameter from the parameter server.");
        }
    }

    void send_raw_data(const std::string& message_name, std::vector<uint8_t> data) {
        if (!message_num_by_name.contains(message_name)) {
            ROS_ERROR("message_name %s is not valid.", message_name.c_str());
        }
        mrover::CAN CANMessage;
        CANMessage.bus = bus;
        MessageID message_id = {0, message_num_by_name[message_name], id};
        CANMessage.message_id = *reinterpret_cast<uint16_t*>(&message_id);
        CANMessage.data = data;
        CANPublisher.publish(CANMessage);
    }

    uint8_t get_id() const {
        return id;
    }

    uint8_t get_bus() const {
        return bus;
    }

private:
    ros::Publisher CANPublisher;
    uint8_t bus{};
    uint8_t id{};
    std::unordered_map<std::string, uint8_t> message_num_by_name;
};