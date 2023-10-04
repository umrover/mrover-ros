#pragma once

#include <cstdint>
#include <map>
#include <string>

#include <XmlRpcValue.h>
#include <ros/ros.h>

#include <mrover/CAN.h>

class CANManager {
public:
    CANManager(ros::NodeHandle& n, const std::string& name) {
        CANPublisher = n.advertise<mrover::CAN>("can_requests", 1);
        std::string can_bus_name = "can/" + name + "/bus";
        assert(n.getParam(can_bus_name, bus));
        std::string can_id_name = "can/" + name + "/id";
        assert(n.getParam(can_id_name, id));
    }

    void send_raw_data(uint64_t frame) {
        ROS_INFO("TODO - need to send %lu based on bus %i.", frame, bus);
    }

    int get_id() const {
        return id;
    }

    int get_bus() const {
        return bus;
    }

private:
    ros::Publisher CANPublisher;
    int bus{};
    int id{};
};