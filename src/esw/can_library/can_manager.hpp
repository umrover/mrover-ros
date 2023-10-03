#pragma once

#include <ros/ros.h>
#include <map>
#include <string>
#include <stdint.h>
#include <mrover/CAN.h>
#include <XmlRpcValue.h>


class CANManager {
public:
    CANManager(ros::NodeHandle* n, std::string& name) {
        CANPublisher = n->advertise<mrover::CAN>("can_requests", 1);
        assert(n->getParam("can/" + name + "/bus", bus));
        assert(n->getParam("can/" + name + "/id", id));
    }

    void send_raw_data(uint64_t frame);

    int get_id() {
        return id;
    }

private:
    ros::Publisher CANPublisher;
    int bus;
    int id;
};